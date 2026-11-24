/**
 * @file bbBFSCode.cpp
 * BFS-tree baseline - see bbBFSCode.hpp.
 *
 * author: Nawar ELZEIN
 **/

#include "bbBFSCode.hpp"

using namespace route;

static const int INF = 1000000;
static const Color VISITED(90, 90, 90);    // trace mode: the module relayed the packet
static const Color DELIVERED(255, 255, 255);
static const Color ROOT(0, 0, 0);          // region representatives are drawn black
// Where a packet is delivered (config: <experiment bfsStop="root|region"/>, default root):
//   root   - the packet climbs the tree all the way to the representative of the target region
//            (classic BFS-to-a-sink; the region's root is the only receiver)
//   region - old behaviour: delivered at the first module of the target region on the way up
static bool stopAtRoot = true;

BbBFSCode::BbBFSCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host), module(host) {
    for (int r = 0; r <= NB_REGIONS; r++) { dist[r] = INF; parent[r] = nullptr; }
    if (not host) return;
    addMessageEventFunc2(BFS_MSG_ID,
        std::bind(&BbBFSCode::onBfsMsg, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(DATA_MSG_ID,
        std::bind(&BbBFSCode::onDataMsg, this, std::placeholders::_1, std::placeholders::_2));
}

void BbBFSCode::parseUserElements(TiXmlDocument *config) {
    parseExperiment(config);
    TiXmlNode *vs = config->FirstChild("vs");
    if (vs) if (TiXmlElement *e = vs->FirstChildElement("experiment"))
        if (const char *a = e->Attribute("bfsStop")) stopAtRoot = std::string(a) != "region";
}

void BbBFSCode::parseUserBlockElements(TiXmlElement *config) {
    if (const char *a = config->Attribute("region")) region = std::stoi(a);
    if (const char *a = config->Attribute("root")) isRoot = Simulator::extractBoolFromString(a);
    if (const char *a = config->Attribute("source")) isSource = Simulator::extractBoolFromString(a);
    if (const char *a = config->Attribute("fail")) willFail = Simulator::extractBoolFromString(a);
    regionOf[module->blockId] = region;
}

void BbBFSCode::startup() {
    if (cfg.allSources) isSource = true;
    if (isRoot) {                       // start the BFS wave of my region's tree
        dist[region] = 0;
        setColor(ROOT);
        sendBfs(region, 0, nullptr);
    }
    if (willFail && cfg.failAt > 0)     // failure test (harness, not part of the algorithm)
        getScheduler()->schedule(new InterruptionEvent<int>(cfg.failAt, module, KILL_EVENT));
    if (isSource)                       // traffic starts after the trees are built
        for (auto &t : trafficPlan(module->blockId, region))
            getScheduler()->schedule(new InterruptionEvent<int>(t.first, module, t.second));
}

void BbBFSCode::sendBfs(int r, int d, P2PNetworkInterface *except) {
    for (auto *itf : module->getP2PNetworkInterfaces()) {
        if (!itf->isConnected() || itf == except) continue;
        sendMessage(new BfsMsg(r, d), itf, 0, 0);
        countSetup(module->blockId, 3, getScheduler()->now());
    }
}

void BbBFSCode::onBfsMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender) {
    if (!isAlive(module)) return;
    BfsMsg *b = static_cast<BfsMsg *>(m.get());
    int cand = b->dist + 1;
    if (cand >= dist[b->root]) return;  // not an improvement: ignore
    dist[b->root] = cand;
    parent[b->root] = sender;           // my way up toward root r
    sendBfs(b->root, cand, sender);
}

void BbBFSCode::forward(int pid, int dstRegion, int hops) {
    P2PNetworkInterface *up = parent[dstRegion];
    if (!up || !up->isConnected()) {   // static tree: a broken parent link means the packet is lost
        noteDrop(module->blockId, pid);
        return;
    }
    sendMessage(new DataMsg(pid, packets[pid].src, packets[pid].srcRegion, dstRegion, hops), up, cfg.hopDelay, 0);
    countData(module->blockId, pid);
}

// traffic start: one packet to every other region (or only to traceDst)
void BbBFSCode::onInterruptionEvent(std::shared_ptr<Event> event) {
    Time now = getScheduler()->now();
    int only = static_cast<InterruptionEvent<int> *>(event.get())->data;   // 0 = all regions
    if (only == KILL_EVENT) { setColor(RED); kill(module); return; }
    if (!isAlive(module)) return;      // a broken module sends nothing
    for (int r = 1; r <= NB_REGIONS; r++) {
        if (r == region) continue;
        if (only > 0 && r != only) continue;
        if (cfg.traceDst > 0 && r != cfg.traceDst) continue;
        int pid = newPacket(module->blockId, region, r, now);
        forward(pid, r, 0);
    }
    if (cfg.traceDst > 0) setColor(VISITED);
}

void BbBFSCode::onDataMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender) {
    if (!isAlive(module)) return;
    DataMsg *d = static_cast<DataMsg *>(m.get());
    int h = d->hops + 1;
    logEv("visit", module->blockId, d->pid, h);
    bool arrived = stopAtRoot ? (isRoot && region == d->dstRegion)   // the root of the target region
                              : (region == d->dstRegion);            // any module of the target region
    if (arrived) {
        deliver(d->pid, module->blockId, h, getScheduler()->now());
        if (cfg.traceDst > 0) setColor(DELIVERED);
        return;
    }
    if (cfg.traceDst > 0 && !isRoot) setColor(VISITED);
    forward(d->pid, d->dstRegion, h);
}

std::string BbBFSCode::onInterfaceDraw() {
    std::stringstream s;
    s << "BFS TREES (to the " << (stopAtRoot ? "root" : "region") << ")  setup msgs=" << cnt.setupMsgs << "  packets=" << packets.size()
      << "  data msgs=" << cnt.dataMsgs;
    return s.str();
}

void BbBFSCode::onEndOfSimulation() {
    std::map<bID, long> mem;
    for (auto &b : BaseSimulator::getWorld()->getMap())
        if (isAlive(b.second)) mem[b.first] = static_cast<BbBFSCode *>(b.second->blockCode)->tableEntries();
    printReport("bfs", mem);
}
