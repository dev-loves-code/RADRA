/**
 * @file bbBFSCode.hpp
 * Baseline 2 - BFS-TREE routing to a REGION (one representative per region).
 *
 * SETUP. Each region has ONE representative module (root="true" in the config,
 * the module nearest the region's centroid). Every root starts its own BFS
 * wave: it sends BFS(r, 0) to its neighbours. A module that receives BFS(r, d)
 * computes d + 1; if this is SMALLER than its current distance to root r it
 * keeps it, takes the sender as its parent toward r, and sends BFS(r, d + 1)
 * to its other neighbours. Keeping only improvements (not the first arrival)
 * makes the result a true shortest-path (BFS) tree even when messages travel
 * at different speeds. The six trees are built at the same time.
 *
 * ROUTING. To reach region r a packet climbs the tree of root r: each module
 * forwards it to its parent toward r, ALL THE WAY TO THE ROOT: the root of
 * region r is the receiver (it does not stop when it enters region r).
 * <experiment bfsStop="region"/> restores the old rule (delivered at the first
 * module of region r met on the way up).
 *
 * State per module: one entry per region = (distance to root, parent face).
 *
 * author: Nawar ELZEIN
 **/

#ifndef bbBFSCode_H_
#define bbBFSCode_H_

#include "routeCommon.hpp"

static const int BFS_MSG_ID = 5101;          // tree construction wave

// BFS(r, d): "root r is d hops away through me"  (region 1 byte + distance 2 bytes)
class BfsMsg : public Message {
public:
    int root; int dist;
    BfsMsg(int r, int d) : Message(), root(r), dist(d) { type = BFS_MSG_ID; }
    unsigned int size() const override { return 3; }
    Message *clone() const override { return new BfsMsg(root, dist); }
};

class BbBFSCode : public BlinkyBlocksBlockCode {
private:
    BlinkyBlocksBlock *module = nullptr;
    int region = 0;
    bool isRoot = false;
    bool isSource = false;
    bool willFail = false;               // failure test: this module breaks at cfg.failAt
    // routing table: one entry per region tree
    int dist[route::NB_REGIONS + 1];
    P2PNetworkInterface *parent[route::NB_REGIONS + 1];

    void sendBfs(int r, int d, P2PNetworkInterface *except);
    void forward(int pid, int dstRegion, int hops);

public:
    BbBFSCode(BlinkyBlocksBlock *host);
    ~BbBFSCode() {};

    void startup() override;
    void parseUserElements(TiXmlDocument *config) override;
    void parseUserBlockElements(TiXmlElement *config) override;
    void onInterruptionEvent(std::shared_ptr<Event> event) override;
    void onBfsMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender);
    void onDataMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender);
    void onEndOfSimulation() override;
    std::string onInterfaceDraw() override;

    long tableEntries() const {
        long n = 0; for (int r = 1; r <= route::NB_REGIONS; r++) if (dist[r] < 1000000) n++; return n;
    }

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new BbBFSCode((BlinkyBlocksBlock *)host));
    }
};

#endif
