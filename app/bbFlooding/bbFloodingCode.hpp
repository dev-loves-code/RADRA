/**
 * @file bbFloodingCode.hpp
 * Baseline 1 - FLOODING with duplicate suppression, routing to a REGION.
 *
 * No setup phase and no routing table. To reach region r, the source sends the
 * packet to all its neighbours; every module that receives a packet id for the
 * FIRST time records it and forwards it to all its neighbours except the one it
 * came from; later copies of the same id are dropped. The packet is delivered
 * (region anycast) at the first module of region r it reaches; the other copies
 * keep spreading until the whole robot has seen the packet, because a plain
 * flood has no way to know the target was reached.
 *
 * State per module: the set of packet ids already seen (grows with traffic).
 * Cost per packet: one transmission on every link in both directions except
 * along the tree of first arrivals = 2E - (N - 1).
 *
 * author: Nawar ELZEIN
 **/

#ifndef bbFloodingCode_H_
#define bbFloodingCode_H_

#include <unordered_set>
#include "routeCommon.hpp"

class BbFloodingCode : public BlinkyBlocksBlockCode {
private:
    BlinkyBlocksBlock *module = nullptr;
    int region = 0;
    bool isSource = false;
    bool willFail = false;               // failure test: this module breaks at cfg.failAt
    std::unordered_set<int> seen;          // packet ids already forwarded (duplicate suppression)

    void flood(int pid, int dstRegion, int hops, P2PNetworkInterface *except);

public:
    BbFloodingCode(BlinkyBlocksBlock *host);
    ~BbFloodingCode() {};

    void startup() override;
    void parseUserElements(TiXmlDocument *config) override;
    void parseUserBlockElements(TiXmlElement *config) override;
    void onInterruptionEvent(std::shared_ptr<Event> event) override;
    void onDataMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender);
    void onEndOfSimulation() override;
    std::string onInterfaceDraw() override;

    size_t seenSize() const { return seen.size(); }

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new BbFloodingCode((BlinkyBlocksBlock *)host));
    }
};

#endif
