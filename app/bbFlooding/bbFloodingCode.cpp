/**
 * @file bbFloodingCode.cpp
 * Flooding baseline - see bbFloodingCode.hpp.
 *
 * author: Nawar ELZEIN
 **/

#include "bbFloodingCode.hpp"

using namespace route;

static const Color VISITED(90, 90, 90);    // trace mode: the module saw the packet
static const Color DELIVERED(255, 255, 255);

BbFloodingCode::BbFloodingCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host), module(host) {
    if (not host) return;
    addMessageEventFunc2(DATA_MSG_ID,
        std::bind(&BbFloodingCode::onDataMsg, this, std::placeholders::_1, std::placeholders::_2));
}

void BbFloodingCode::parseUserElements(TiXmlDocument *config) { parseExperiment(config); }

void BbFloodingCode::parseUserBlockElements(TiXmlElement *config) {
    if (const char *a = config->Attribute("region")) region = std::stoi(a);
    if (const char *a = config->Attribute("source")) isSource = Simulator::extractBoolFromString(a);
    if (const char *a = config->Attribute("fail")) willFail = Simulator::extractBoolFromString(a);
    regionOf[module->blockId] = region;
}

void BbFloodingCode::startup() {
    if (cfg.allSources) isSource = true;
    // flooding needs no setup: the only scheduled event is the traffic start
    // (same start date as the BFS app, so both runs see the same traffic)
    if (willFail && cfg.failAt > 0)     // failure test (harness, not part of the algorithm)
        getScheduler()->schedule(new InterruptionEvent<int>(cfg.failAt, module, KILL_EVENT));
    if (isSource)
        for (auto &t : trafficPlan(module->blockId, region))
            getScheduler()->schedule(new InterruptionEvent<int>(t.first, module, t.second));
}

// send one copy of the packet on every connected face except 'except'
void BbFloodingCode::flood(int pid, int dstRegion, int hops, P2PNetworkInterface *except) {
    for (auto *itf : module->getP2PNetworkInterfaces()) {
        if (!itf->isConnected() || itf == except) continue;
        sendMessage(new DataMsg(pid, packets[pid].src, packets[pid].srcRegion, dstRegion, hops), itf, cfg.hopDelay, 0);
        countData(module->blockId, pid);
    }
}

// traffic start: one packet to every other region (or only to traceDst)
void BbFloodingCode::onInterruptionEvent(std::shared_ptr<Event> event) {
    Time now = getScheduler()->now();
    int only = static_cast<InterruptionEvent<int> *>(event.get())->data;   // 0 = all regions
    if (only == KILL_EVENT) { setColor(RED); kill(module); return; }
    if (!isAlive(module)) return;      // a broken module sends nothing
    for (int r = 1; r <= NB_REGIONS; r++) {
        if (r == region) continue;
        if (only > 0 && r != only) continue;
        if (cfg.traceDst > 0 && r != cfg.traceDst) continue;
        int pid = newPacket(module->blockId, region, r, now);
        seen.insert(pid);
        flood(pid, r, 0, nullptr);
    }
    if (cfg.traceDst > 0) setColor(VISITED);
}

void BbFloodingCode::onDataMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender) {
    if (!isAlive(module)) return;
    DataMsg *d = static_cast<DataMsg *>(m.get());
    if (!seen.insert(d->pid).second) return;          // duplicate: drop
    int h = d->hops + 1;
    logEv("visit", module->blockId, d->pid, h);
    Time now = getScheduler()->now();
    bool first = false;
    if (region == d->dstRegion) first = deliver(d->pid, module->blockId, h, now);
    if (cfg.traceDst > 0) setColor(first ? DELIVERED : VISITED);
    flood(d->pid, d->dstRegion, h, sender);            // plain flooding: everyone forwards once
}

std::string BbFloodingCode::onInterfaceDraw() {
    std::stringstream s;
    s << "FLOODING  packets=" << packets.size() << "  data msgs=" << cnt.dataMsgs;
    return s.str();
}

void BbFloodingCode::onEndOfSimulation() {
    std::map<bID, long> mem;
    for (auto &b : BaseSimulator::getWorld()->getMap())
        if (isAlive(b.second)) mem[b.first] = (long)static_cast<BbFloodingCode *>(b.second->blockCode)->seenSize();
    printReport("flooding", mem);
}
