/**
 * @file routeCommon.hpp
 * Shared by bbFlooding and bbBFS (copied into each app so each one builds on
 * its own, as in the course labs).
 *
 *  - Experiment settings read from the config  (<experiment .../>, per-block
 *    region / root / source attributes)
 *  - The data packet both algorithms carry       (DataMsg, 16 bytes)
 *  - Statistics: every transmission is counted per message kind and per module;
 *    every packet is followed from its source to its first delivery
 *  - An OFFLINE shortest-distance oracle (multi-source BFS over the world map),
 *    used only at the end of the run to measure path stretch. It is never
 *    used by the algorithms themselves.
 *
 * author: Nawar ELZEIN
 **/

#ifndef ROUTE_COMMON_HPP_
#define ROUTE_COMMON_HPP_

#include <map>
#include <set>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <functional>
#include <cstdlib>

#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"
#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"

using namespace BlinkyBlocks;

namespace route {

static const int NB_REGIONS = 6;
static const char *REGION_NAME[NB_REGIONS + 1] =
    {"-", "head", "body", "left arm", "right arm", "left leg", "right leg"};

static const int DATA_MSG_ID = 5001;       // the routed data packet (both algorithms)
static const unsigned DATA_SIZE = 16;      // bytes: header (src 2, pid 4, regions 2, hops 1) + 7 payload

// ---------------------------------------------------------------- experiment
struct Experiment {
    std::string name = "unnamed";
    Time start = 3000000;      // us: traffic starts here (after any setup phase)
    bool allSources = true;    // every module sends, or only blocks marked source="true"
    int traceDst = -1;         // trace mode: one marked source sends ONE packet to this region
    Time spacing = 0;          // 0 = every source sends all its packets at 'start' (heavy load);
                               // >0 = packets leave one by one, 'spacing' us apart (light load:
                               //      no two packets are ever in the robot at the same time)
    Time hopDelay = 0;         // extra delay per data hop, ONLY to make trace runs watchable in the GUI
    Time failAt = 0;           // failure test: blocks marked fail="true" are cut out at this date (0 = none)
};
inline Experiment cfg;

inline void parseExperiment(TiXmlDocument *config) {
    TiXmlNode *vs = config->FirstChild("vs");
    if (!vs) return;
    TiXmlElement *e = vs->FirstChildElement("experiment");
    if (!e) return;
    if (const char *a = e->Attribute("name")) cfg.name = a;
    if (const char *a = e->Attribute("start")) cfg.start = std::stoull(a);
    if (const char *a = e->Attribute("sources")) cfg.allSources = (std::string(a) == "all");
    if (const char *a = e->Attribute("traceDst")) cfg.traceDst = std::stoi(a);
    if (const char *a = e->Attribute("spacing")) cfg.spacing = std::stoull(a);
    if (const char *a = e->Attribute("hopDelay")) cfg.hopDelay = std::stoull(a);
    if (const char *a = e->Attribute("failAt")) cfg.failAt = std::stoull(a);
}

// Traffic plan, identical for both algorithms. Each source sends one packet to
// every other region (trace mode: only to traceDst). Returns (date, dstRegion)
// pairs; dstRegion = 0 means "all my packets now" (heavy load).
inline std::vector<std::pair<Time, int>> trafficPlan(bID id, int myRegion) {
    std::vector<std::pair<Time, int>> plan;
    if (cfg.spacing == 0) { plan.push_back({cfg.start, 0}); return plan; }
    int j = 0;
    for (int r = 1; r <= NB_REGIONS; r++) {
        if (r == myRegion) continue;
        if (cfg.traceDst > 0 && r != cfg.traceDst) continue;
        plan.push_back({cfg.start + (Time)((id - 1) * (NB_REGIONS - 1) + j) * cfg.spacing, r});
        j++;
    }
    return plan;
}

// per-block attributes, kept globally for the statistics and the oracle
inline std::map<bID, int> regionOf;

// ---------------------------------------------------------------- failures (experiment harness)
// A module marked fail="true" schedules an interruption with this value; when it
// fires, the harness disconnects the module from all its neighbours (as if it
// broke down). Each neighbour is then told by VisibleSim itself, through
// onNeighborChanged(face, EVENT_REMOVE_NEIGHBOR): detection is local, no timers.
static const int KILL_EVENT = -1;

// ---------------------------------------------------------------- event log (visuals only)
// When the environment variable RR_LOG names a file, every event is written to it
// as "time_us,event,module,pid,value". It is read by tools/animate.py to replay a run.
// Events: ctl (gradient/tree message sent), tx (data message sent), visit (module
// handles a data packet, value = hops), deliver, drop, kill. No effect on the algorithms.
inline std::ofstream *evlog = nullptr;
inline bool evlogTried = false;
inline void logEv(const char *ev, bID id, int pid = -1, int value = -1) {
    if (!evlogTried) {
        evlogTried = true;
        if (const char *f = std::getenv("RR_LOG")) { evlog = new std::ofstream(f); *evlog << "t,ev,id,pid,v\n"; }
    }
    if (evlog) *evlog << BaseSimulator::getScheduler()->now() << "," << ev << "," << id << "," << pid << "," << value << "\n";
}
inline std::vector<bID> killed;
inline bool isAlive(BuildingBlock *b) { return b->getState() >= BuildingBlock::ALIVE; }
inline void kill(BuildingBlock *b) {
    if (!isAlive(b)) return;
    BaseSimulator::getWorld()->disconnectBlock(b, false);
    b->setState(BuildingBlock::REMOVED);
    killed.push_back(b->blockId);
    logEv("kill", b->blockId);
}

// ---------------------------------------------------------------- data packet
class DataMsg : public Message {
public:
    int pid; bID src; int srcRegion; int dstRegion; int hops;
    DataMsg(int p, bID s, int sr, int dr, int h)
        : Message(), pid(p), src(s), srcRegion(sr), dstRegion(dr), hops(h) { type = DATA_MSG_ID; }
    unsigned int size() const override { return DATA_SIZE; }
    Message *clone() const override { return new DataMsg(pid, src, srcRegion, dstRegion, hops); }
};

// ---------------------------------------------------------------- statistics
struct PacketRecord {
    bID src = 0; int srcRegion = 0; int dstRegion = 0;
    Time sent = 0, delivered = 0;
    bool ok = false; int hops = -1; bID by = 0;
    long tx = 0;                       // every transmission of this packet, anywhere
};
inline std::vector<PacketRecord> packets;

struct Counters {
    long setupMsgs = 0, setupBytes = 0;      // control messages before the failure (or all, if none)
    long repairMsgs = 0, repairBytes = 0;    // control messages sent after the failure date
    long drops = 0;                          // packets dropped: no usable next hop
    Time lastRepair = 0;
    long dataMsgs = 0, dataBytes = 0;
    Time lastSetup = 0;
    std::map<bID, long> txPerModule;   // all messages sent by each module (load / energy)
    std::map<bID, long> repairPerModule; // control messages sent after the failure, per module
};
inline Counters cnt;

inline int newPacket(bID src, int sr, int dr, Time now) {
    PacketRecord r; r.src = src; r.srcRegion = sr; r.dstRegion = dr; r.sent = now;
    packets.push_back(r);
    return (int)packets.size() - 1;
}

inline void countSetup(bID from, unsigned bytes, Time now) {
    cnt.txPerModule[from]++;
    logEv("ctl", from);
    if (cfg.failAt > 0 && now >= cfg.failAt) {
        cnt.repairMsgs++; cnt.repairBytes += bytes; cnt.repairPerModule[from]++;
        if (now > cnt.lastRepair) cnt.lastRepair = now;
        return;
    }
    cnt.setupMsgs++; cnt.setupBytes += bytes;
    if (now > cnt.lastSetup) cnt.lastSetup = now;
}

inline void countData(bID from, int pid) {
    cnt.dataMsgs++; cnt.dataBytes += DATA_SIZE; cnt.txPerModule[from]++;
    packets[pid].tx++;
    logEv("tx", from, pid);
}

inline void noteDrop(bID at, int pid = -1) { cnt.drops++; logEv("drop", at, pid); }

// first arrival in the destination region = delivery (region anycast)
inline bool deliver(int pid, bID by, int hops, Time now) {
    PacketRecord &r = packets[pid];
    if (r.ok) return false;
    r.ok = true; r.hops = hops; r.by = by; r.delivered = now;
    logEv("deliver", by, pid, hops);
    return true;
}

// ---------------------------------------------------------------- oracle
// dist[r][id] = true shortest hop count from module id to the nearest module
// of region r, computed offline on the final world graph (statistics only).
inline std::map<bID, int> oracleDist[NB_REGIONS + 1];

inline void computeOracle() {
    auto &blocks = BaseSimulator::getWorld()->getMap();
    for (int r = 1; r <= NB_REGIONS; r++) {
        std::deque<bID> q;
        for (auto &b : blocks)
            if (isAlive(b.second) && regionOf[b.first] == r) { oracleDist[r][b.first] = 0; q.push_back(b.first); }
        while (!q.empty()) {
            bID u = q.front(); q.pop_front();
            BuildingBlock *bb = BaseSimulator::getWorld()->getBlockById(u);
            for (auto *itf : bb->getP2PNetworkInterfaces()) {
                if (!itf->isConnected()) continue;
                bID v = itf->getConnectedBlockId();
                if (oracleDist[r].count(v)) continue;
                oracleDist[r][v] = oracleDist[r][u] + 1;
                q.push_back(v);
            }
        }
    }
}

inline long countLinks() {
    long half = 0;
    for (auto &b : BaseSimulator::getWorld()->getMap())
        if (isAlive(b.second))
            for (auto *itf : b.second->getP2PNetworkInterfaces()) if (itf->isConnected()) half++;
    return half / 2;
}

// ---------------------------------------------------------------- report
// memEntries: routing-state entries held by each module at the end of the run
inline void printReport(const std::string &algo, const std::map<bID, long> &memEntries) {
    computeOracle();
    auto &blocks = BaseSimulator::getWorld()->getMap();
    long N = 0; for (auto &b : blocks) if (isAlive(b.second)) N++;   // surviving modules
    long E = countLinks();
    long okCount = 0, deliverable = 0; double sumH = 0, sumO = 0, sumStretch = 0; int maxH = 0, maxO = 0;
    double sumLat = 0; Time maxLat = 0;
    std::string csv = "results_" + algo + "_" + cfg.name + ".csv";
    std::ofstream f(csv);
    f << "pid,src,srcRegion,dstRegion,hops,oracle,tx,latency_us,deliveredBy\n";
    for (size_t i = 0; i < packets.size(); i++) {
        PacketRecord &p = packets[i];
        // oracle on the FINAL graph; -1 = region unreachable from the source (physically cut off)
        int o = oracleDist[p.dstRegion].count(p.src) ? oracleDist[p.dstRegion][p.src] : -1;
        if (o >= 0) deliverable++;
        f << i << "," << p.src << "," << p.srcRegion << "," << p.dstRegion << ","
          << p.hops << "," << o << "," << p.tx << ","
          << (p.ok ? (long long)(p.delivered - p.sent) : -1) << "," << p.by << "\n";
        if (!p.ok) continue;
        okCount++; sumH += p.hops; sumO += o; sumStretch += (o > 0 ? double(p.hops) / o : 1.0);
        maxH = std::max(maxH, p.hops); maxO = std::max(maxO, o);
        Time lat = p.delivered - p.sent; sumLat += lat; maxLat = std::max(maxLat, lat);
    }
    f.close();
    if (evlog) evlog->flush();
    long maxTx = 0; bID hot = 0; double sumTx = 0;
    for (auto &b : blocks) { if (!isAlive(b.second)) continue;
                             long t = cnt.txPerModule.count(b.first) ? cnt.txPerModule.at(b.first) : 0;
                             sumTx += t; if (t > maxTx) { maxTx = t; hot = b.first; } }
    std::ofstream fm("modules_" + algo + "_" + cfg.name + ".csv");
    fm << "id,x,y,z,region,tx,memEntries,alive,repairTx\n";
    for (auto &b : blocks) {
        const Cell3DPosition &p = b.second->position;
        fm << b.first << "," << p[0] << "," << p[1] << "," << p[2] << "," << regionOf[b.first] << ","
           << (cnt.txPerModule.count(b.first) ? cnt.txPerModule.at(b.first) : 0) << ","
           << (memEntries.count(b.first) ? memEntries.at(b.first) : 0) << "," << isAlive(b.second) << ","
           << (cnt.repairPerModule.count(b.first) ? cnt.repairPerModule.at(b.first) : 0) << "\n";
    }
    fm.close();
    long maxMem = 0; double sumMem = 0;
    for (auto &m : memEntries) { maxMem = std::max(maxMem, m.second); sumMem += m.second; }
    double P = packets.size();
    std::cout << std::fixed;
    std::cout.precision(3);
    std::cout << "RESULT algo=" << algo << " config=" << cfg.name
              << " N=" << N << " E=" << E << " packets=" << packets.size()
              << " delivered=" << okCount << " deliverable=" << deliverable
              << " killed=" << killed.size() << " drops=" << cnt.drops
              << " setupMsgs=" << cnt.setupMsgs << " setupBytes=" << cnt.setupBytes
              << " repairMsgs=" << cnt.repairMsgs << " repairBytes=" << cnt.repairBytes
              << " dataMsgs=" << cnt.dataMsgs << " dataBytes=" << cnt.dataBytes
              << " totalMsgs=" << (cnt.setupMsgs + cnt.repairMsgs + cnt.dataMsgs)
              << " dataMsgsPerPacket=" << (P > 0 ? cnt.dataMsgs / P : 0)
              << " avgHops=" << (okCount ? sumH / okCount : 0)
              << " avgOracle=" << (okCount ? sumO / okCount : 0)
              << " avgStretch=" << (okCount ? sumStretch / okCount : 0)
              << " maxHops=" << maxH << " maxOracle=" << maxO
              << " avgLatency_ms=" << (okCount ? sumLat / okCount / 1000.0 : 0)
              << " maxLatency_ms=" << maxLat / 1000.0
              << " avgTxPerModule=" << sumTx / N << " maxTxPerModule=" << maxTx << " hotModule=" << hot
              << " avgMemEntries=" << sumMem / N << " maxMemEntries=" << maxMem
              << " setupEnd_ms=" << cnt.lastSetup / 1000.0
              << " repairEnd_ms=" << cnt.lastRepair / 1000.0
              << " failAt_ms=" << cfg.failAt / 1000.0
              << " trafficStart_ms=" << cfg.start / 1000.0
              << " csv=" << csv << "\n";
    if (cnt.setupMsgs > 0 && cnt.lastSetup >= cfg.start)
        std::cout << "WARNING setup was still running when traffic started\n";
    if (cnt.repairMsgs > 0 && cnt.lastRepair >= cfg.start)
        std::cout << "WARNING repair was still running when traffic started\n";
    for (int r = 1; r <= NB_REGIONS; r++) {
        int n = 0; for (auto &b : blocks) if (isAlive(b.second) && regionOf[b.first] == r) n++;
        std::cout << "REGION R" << r << " " << REGION_NAME[r] << " modules=" << n << "\n";
    }
}

} // namespace route

#endif
