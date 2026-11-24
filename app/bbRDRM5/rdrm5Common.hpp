/**
 * @file rdrm5Common.hpp
 * Experiment harness for bbRDRM5 (= bbRDRM3 + load/energy levels carried by the heartbeats;
 * derived from routeCommon.hpp of the baselines):
 * settings, traffic plan (anycast or unicast), SILENT failures, statistics by
 * message kind, memory in bytes, offline oracle.
 *
 * author: Nawar ELZEIN
 **/

#ifndef RDRM5_COMMON_HPP_
#define RDRM5_COMMON_HPP_

#include <map>
#include <set>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>

#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"
#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"

using namespace BlinkyBlocks;

namespace r2 {

static const int NB_REGIONS = 6;

// ---------------------------------------------------------------- experiment
struct Experiment {
    std::string name = "unnamed";
    Time start = 5000000;          // traffic starts (after gradient + map / table setup)
    Time spacing = 20000;          // one packet every 'spacing' us, robot-wide
    Time hopDelay = 0;             // GUI traces only
    bool allSources = true;
    int traceDst = -1;
    std::string traffic = "unicast";   // anycast | unicast
    std::string targets = "uniform";   // uniform: any cube of the region | hot:n: n fixed cubes per region
    int hotN = 3;
    std::string inreg = "map";     // flood | dv | mapbfs | route | geo (textbook, no map) | adapt (geo when the map says it is sure) | map | mapgeo (older variants)
    std::string detect = "hb";     // hb (heartbeats) | event (simulator alert) | none
    Time hbPeriod = 500000;        // heartbeat period
    Time mapDelBatch = 0;          // >0: pass map deletions on in bundles, at most this long after the first (0 = one message per cell)
    std::string falseFix = "off";  // on: a "dead" neighbour heard again is put back on the map (and the region told); that link then waits hbMissExtra more beats
    int hbMissExtra = 3;
    std::string linkAck = "off";   // on: per-link sequence numbers; the heartbeat back says which arrived and which were passed on:
                                   //     passed-on copies are dropped (no duplicate resend after a death), lost ones resent over the same link
    Time lossWait = 1500000;
    Time gapWait = 400000;         // linkAck: a gap (later ones arrived) is resent only if the copy went out at least this long ago       // linkAck: a copy never confirmed as received after this long is resent over the link (tail loss)
    std::string baseline = "none"; // state-of-the-art baselines: "lengo" (one gradient per region, from its root only: Le & Ngo 2016) | "gpsr" (robot-wide greedy + right-hand perimeter, no gradient)
    std::string hbAdapt = "off";   // on: fast heartbeats only on links in use (a neighbour depends on me, or data passed in the last few s); slow elsewhere
    Time hbSlow = 2000000;         // hbAdapt: period on idle links (detection there takes hbMiss x hbSlow)
    Time hbBusy = 3000000;
    std::string hbScale = "off";   // on: busy window >= 4 periods and slow period >= 2 periods, whatever hbPeriod is         // hbAdapt: a link stays "in use" this long after data (the watcher expects fast beats 0.5 s less)
    std::string hbScope = "parents";   // parents: a parent beats to its children | all: every cube beats to every neighbour
    std::string cti = "count";     // count: distances climb one step at a time (count-to-infinity, capped at dmax)
                                   // poison: a distance never climbs; "impossible" at once, then a hold-down
    Time hold = 300000;            // poison: how long an "impossible" cube ignores old values
    std::string leader = "config"; // config: the map wave starts at root="true" | elect: echo with extinction (lowest id wins)
    int hbMiss = 3;                // parent declared dead after this many missed periods
    bool hbPiggy = true;           // skip a heartbeat on a face that carried another message this period (v4: on;
                                   // with sig=hb only when that neighbour already knows my current levels)
    bool mergeSub = true;          // subscriptions ride on the gradient ADVs            (merge="sub" or "all")
    bool mergeEcho = false;        // the map wave rides on the region's own ADV(r,0)     (merge="all")
    Time failAt = 0;               // 0 = no failure
    std::string failMode = "silent";   // silent: the cube stops; links stay | disconnect: the simulator unplugs it
    Time mapAt = 1000000;          // in-region setup (map echo / dv tables) starts
    int dmax = 255;
    int cacheSize = 16;            // route mode: routes kept by an entry cube
    Time stopAt = 0;               // computed: end of heartbeats
    // ---- v3: load balancing and energy (all off by default = v2 behaviour)
    std::string balance = "none";  // none: lowest id among closer neighbours (v2) | rr: blind round robin among them
                                   // aware: state-aware choice among them | detour: aware + bounded sideways detours
    double theta = 0.6;            // load threshold: below it a link/neighbour costs nothing ("not broken")
    double thetaE = 0.5;           // energy threshold: below it a neighbour starts to cost
    double deltaE = 2.0;           // weight of the energy deficit (linear): at deltaE*deficit = lambda a detour pays
    double hyst = 0.15;            // switch next hop only if the gain exceeds this (no flapping)
    double lambda = 1.0;           // cost of ONE extra hop, in penalty units (detour only if it saves more)
    int detourMax = 2;             // sideways hops a packet may take
    Time statePeriod = 100000;     // load/energy measurement tick; a STATE message only when a level changes
    double ewma = 0.5;             // smoothing of the link utilisation
    std::string energy = "off";    // on: battery per module, a module dies when it is empty
    double battery = 2e6;          // uJ per module
    double eTx = 10, eRx = 5;      // uJ per byte sent / received
    double lowPct = 0, lowLevel = 0.3; int lowSeed = 1;   // batteryLow="pct:level:seed": some modules start weak
    bool bye = true;               // energy: a module about to run out says goodbye (its neighbours route around at once)
    double reserve = 0.03;         // battery share kept to say goodbye
    std::string hotSrc = "3456";   // traffic="hotspot": source regions
    int hotDst = 1;                // traffic="hotspot": target region (anycast)
    int hotCount = 10;             // packets per source
    int geoEvery = 5;              // traffic="geocast": one module in geoEvery sends one geocast to another region
    Time hotPeriod = 400000;       // per-source period (aggregate rate = sources / hotPeriod)
    // ---- v4: how load/energy levels travel
    std::string sig = "hb";        // hb: inside the heartbeats (v4) | state: separate STATE messages to every neighbour (v3)
    int deadband = 2;
    // ---- v5: recovery buffer (no extra control messages: only the packets that may have died are sent again)
    std::string salvage = "on";    // on: keep a copy of the data packets sent on each face in the last salvageWin;
                                   //     when that neighbour is declared dead, route them again (the destination drops duplicates)
    Time salvageWin = 4000000;     // how far back the copies go (us): two failures in a row on one path (2 x 1.5 s detection + margin)
    int salvageMax = 128;          // copies per face (oldest dropped first). 128 ~ what a link carries (174 pkt/s) before suspect (0.75 s) stops sending to a quiet neighbour
    Time parkRetry = 100000;       // a packet with no route right now (hold-down) waits here and is retried every parkRetry
    Time parkTimeout = 1500000;    // ...for at most this long, then it is dropped
    int parkMax = 256;             // waiting packets per module: one face's worth of resent copies can arrive at once
    double linkLoss = 0;           // test: each message is lost on the wire with this probability (both modules stay alive)
    int lossSeed = 7; Time lossFrom = 0;   // loss starts at this date (0 = from power-on)
    std::string split = "none";    // a region torn in pieces: none | grad (every piece announces its own gradient) | seek (pieces found on demand)
    int localPct = 100, localAdd = 6;   // local: a piece announcement travels at most pct % of its region size (bbox dx+dy+dz) + add hops
    Time splitSettle = 300000;     // grad: wait this long after the last map change before announcing my piece
    int seekTries = 1;             // seek: rings tried (TTL x2 each try)
    int seekTtlPct = 100, seekTtlAdd = 4;
    int walkPct = 400, walkAdd = 20;
    int walkTries = 2; Time walkBackoff = 2000000;   // walk: tries (aims alternate; budget x1, x2), then the backup, then a pause
    Time floodBackoff = 20000000;  // walk: after an unanswered flooded question, that cell is not asked for again for this long
    std::string walkFallback = "seek";   // walk: after the walks, one flooded question (2 x DMAX hops): always finds a real piece
    std::string quietPark = "off"; // on: data never goes to a quiet neighbour (waits instead)
    Time quietWait = 2500000;      // a packet held back from a quiet neighbour waits this long (detection + repair)
    Time splitWait = 5000000;      // a packet waiting for the way to another piece waits this long (others: parkTimeout)
    Time gcKeep = 4000000;         // geocast entry module: keeps its geocasts this long, to resend them into pieces found later   // walk: at most pct % of the straight distance + add hops   // seek: first ring = pct % of the straight distance to the target + add hops
    Time seekBackoff = 5000000;    // seek: a piece that did not answer is not asked again for this long
    Time seekPerHop = 12000;       // seek: time allowed per hop of TTL before trying a wider ring
    std::string piece = "off";     // salvage on: a piece of a region cut off from its main part stops posing as the region (see computePiece)
    std::string escape = "off";    int escapeBudget = 200;   // escape: at most escapeBudget % of the distance to the target + 10 hops, then give up
       // salvage on: a target cut off inside its own region is reached around the cut, through any module (geo)
    std::string widen = "on";      // local: a module that cannot hear a piece of its torn region doubles the announcement reach (up to the whole robot)
    Time widenAfter = 1500000;     // ...after this long without hearing it
    std::string e2e = "off";       // on: the source keeps every packet until the target acknowledges it (end-to-end), resends otherwise
    Time rto = 3000000, rtoMax = 16000000;
    std::string rtoAdapt = "off";  // on: first resend after srtt + 4 rttvar of my own receipts (Jacobson/Karn), kept in [rtoMin, rtoCap]
    Time rtoMin = 2000000, rtoCap = 6000000;   // e2e: first resend after rto, then x2 each time, at most rtoMax apart
    int e2eTries = 10;             // e2e: transmissions before the source gives up (lifetime ~ 2 min)
    std::string copyLog = "on";    // off: no per-hop copies (the waiting room stays); test of e2e alone
    int reliablePct = 100;         // e2e: share of packets that ask for a receipt (the "reliable" flag; the rest are best effort)
    std::string ackCopy = "on";    // off: no per-hop copies of receipts (a lost receipt only costs one resend)
    Time goneWait = 0;             // e2e: answer "gone" only for a cell erased at least this long ago (a false alarm is undone within ~1-2 s): wait meanwhile
    int goneRetries = 0;           // e2e: after a "gone" answer, try again this many times (rtoMax apart): a target declared dead by mistake comes back
    std::string unreach = "off";   // e2e: a packet that waited for a piece not heard even at full reach -> "unreachable"; the source stops after two such answers >= rtoMax apart
    std::string gcAck = "off";     // e2e: each geocast copy sent to another piece of a torn region is acknowledged by that piece; the entry module resends
    std::string advSync = "off";   // a lost gradient advert is noticed (1-byte checksum on every 4th heartbeat) and the neighbour resends its distances
    std::string anyDedup = "off";  // anycast: a module that already delivered a packet drops later copies of it (answers again)
    std::string giveUp = "off";    // (off: had false alarms in cascades) a piece not heard even across the whole robot (twice) is out of reach: drop its packets, tell the source
    std::string nack = "on";
    std::string noway = "off";     // e2e: a packet that expires with no way tells its source (measured: no gain, many messages)       // e2e: the target region answers "gone" when its map says the target cell is dead
    Time tail = 0;                 // test: keep the robot running this long after the last packet (resends need time)
    std::string suspect = "off";   // on: a neighbour silent for suspectAfter is avoided for data when another closer one exists
    Time suspectAfter = 750000;              // hb: a user hears a change within a zone only when the level moved by this much
    Time userWin = 500000;         // hb: a neighbour that sent me data in this window is a user (gets my changes at once)
};
inline Experiment cfg;

inline bool usesMap() { return cfg.inreg == "map" || cfg.inreg == "mapbfs" || cfg.inreg == "mapgeo" || cfg.inreg == "route" || cfg.inreg == "adapt"; }
inline bool mergedEcho() { return cfg.mergeEcho && cfg.traffic == "unicast" && usesMap(); }

inline void parseExperiment(TiXmlDocument *config) {
    TiXmlNode *vs = config->FirstChild("vs");
    if (!vs) return;
    TiXmlElement *e = vs->FirstChildElement("experiment");
    if (!e) return;
    auto S = [&](const char *k, std::string &v) { if (const char *a = e->Attribute(k)) v = a; };
    auto T = [&](const char *k, Time &v) { if (const char *a = e->Attribute(k)) v = std::stoull(a); };
    auto I = [&](const char *k, int &v) { if (const char *a = e->Attribute(k)) v = std::stoi(a); };
    // profile="final" (default): the delivered algorithm; any attribute below still overrides it.
    // profile="legacy": the old defaults (what configs written before the final version expect)
    std::string profile = "final"; S("profile", profile);
    if (profile == "final") {
        cfg.cti = "poison"; cfg.hbScope = "all"; cfg.leader = "elect"; cfg.balance = "detour"; cfg.inreg = "adapt";
        cfg.salvage = "on"; cfg.suspect = "on"; cfg.split = "local"; cfg.widen = "on"; cfg.e2e = "on"; cfg.hbAdapt = "on"; cfg.ackCopy = "off";
        cfg.linkAck = "on"; cfg.falseFix = "on"; cfg.hbScale = "on"; cfg.goneWait = 2000000; cfg.gcAck = "on"; cfg.unreach = "on"; cfg.rto = 5000000; cfg.anyDedup = "on"; cfg.advSync = "on";
    }
    S("name", cfg.name); T("start", cfg.start); T("spacing", cfg.spacing); T("hopDelay", cfg.hopDelay);
    if (const char *a = e->Attribute("sources")) cfg.allSources = (std::string(a) == "all");
    I("traceDst", cfg.traceDst); S("traffic", cfg.traffic); S("targets", cfg.targets); S("inreg", cfg.inreg);
    S("detect", cfg.detect); S("hbAdapt", cfg.hbAdapt); S("hbScale", cfg.hbScale); S("baseline", cfg.baseline); S("linkAck", cfg.linkAck); S("falseFix", cfg.falseFix); I("hbMissExtra", cfg.hbMissExtra); T("lossWait", cfg.lossWait); T("gapWait", cfg.gapWait); T("mapDelBatch", cfg.mapDelBatch); T("hbSlow", cfg.hbSlow); T("hbBusy", cfg.hbBusy); T("hbPeriod", cfg.hbPeriod); I("hbMiss", cfg.hbMiss); S("hbScope", cfg.hbScope); S("cti", cfg.cti); T("hold", cfg.hold); S("leader", cfg.leader); if (const char *a = e->Attribute("hbPiggy")) cfg.hbPiggy = std::string(a) == "1"; if (const char *a = e->Attribute("merge")) { std::string m = a; cfg.mergeSub = (m == "sub" || m == "all"); cfg.mergeEcho = (m == "all"); } T("failAt", cfg.failAt);
    { auto D = [&](const char *k, double &v) { if (const char *a = e->Attribute(k)) v = std::stod(a); };
      S("balance", cfg.balance); D("theta", cfg.theta); D("thetaE", cfg.thetaE); D("deltaE", cfg.deltaE); D("hyst", cfg.hyst); D("lambda", cfg.lambda);
      I("detourMax", cfg.detourMax); T("statePeriod", cfg.statePeriod); D("ewma", cfg.ewma); S("energy", cfg.energy);
      D("battery", cfg.battery); D("eTx", cfg.eTx); D("eRx", cfg.eRx); if (const char *b = e->Attribute("bye")) cfg.bye = std::string(b) == "1"; D("reserve", cfg.reserve); if (const char *b = e->Attribute("batteryLow")) { std::string v = b; auto p1 = v.find(':'), p2 = v.rfind(':');
        cfg.lowPct = std::stod(v.substr(0, p1)); cfg.lowLevel = std::stod(v.substr(p1 + 1, p2 - p1 - 1)); cfg.lowSeed = std::stoi(v.substr(p2 + 1)); } S("hotSrc", cfg.hotSrc); I("hotDst", cfg.hotDst);
      I("hotCount", cfg.hotCount); I("geoEvery", cfg.geoEvery); T("hotPeriod", cfg.hotPeriod);
      S("sig", cfg.sig); I("deadband", cfg.deadband); T("userWin", cfg.userWin);
      S("salvage", cfg.salvage); T("salvageWin", cfg.salvageWin); I("salvageMax", cfg.salvageMax); T("parkRetry", cfg.parkRetry);
      T("parkTimeout", cfg.parkTimeout); I("parkMax", cfg.parkMax); S("escape", cfg.escape); S("split", cfg.split); if (const char *a = e->Attribute("linkLoss")) cfg.linkLoss = std::stod(a); I("lossSeed", cfg.lossSeed); T("lossFrom", cfg.lossFrom); T("splitSettle", cfg.splitSettle); I("localPct", cfg.localPct); I("localAdd", cfg.localAdd); I("seekTries", cfg.seekTries); I("seekTtlPct", cfg.seekTtlPct); I("walkPct", cfg.walkPct); I("walkTries", cfg.walkTries); S("walkFallback", cfg.walkFallback); T("floodBackoff", cfg.floodBackoff); T("walkBackoff", cfg.walkBackoff); T("splitWait", cfg.splitWait); T("quietWait", cfg.quietWait); S("quietPark", cfg.quietPark); T("gcKeep", cfg.gcKeep); I("walkAdd", cfg.walkAdd); I("seekTtlAdd", cfg.seekTtlAdd); T("seekBackoff", cfg.seekBackoff); T("seekPerHop", cfg.seekPerHop); S("piece", cfg.piece); I("escapeBudget", cfg.escapeBudget); S("suspect", cfg.suspect); S("e2e", cfg.e2e); S("widen", cfg.widen); T("widenAfter", cfg.widenAfter); T("rto", cfg.rto); S("rtoAdapt", cfg.rtoAdapt); T("rtoMin", cfg.rtoMin); T("rtoCap", cfg.rtoCap); T("rtoMax", cfg.rtoMax); I("e2eTries", cfg.e2eTries); S("copyLog", cfg.copyLog); S("nack", cfg.nack); S("giveUp", cfg.giveUp); S("gcAck", cfg.gcAck); S("anyDedup", cfg.anyDedup); S("advSync", cfg.advSync); S("unreach", cfg.unreach); I("goneRetries", cfg.goneRetries); T("goneWait", cfg.goneWait); I("reliablePct", cfg.reliablePct); S("ackCopy", cfg.ackCopy); S("noway", cfg.noway); T("tail", cfg.tail); T("suspectAfter", cfg.suspectAfter); }
    S("failMode", cfg.failMode); T("mapAt", cfg.mapAt); I("dmax", cfg.dmax); I("cacheSize", cfg.cacheSize);
    if (cfg.targets.rfind("hot", 0) == 0) { auto p = cfg.targets.find(':'); if (p != std::string::npos) cfg.hotN = std::stoi(cfg.targets.substr(p + 1)); cfg.targets = "hot"; }
}

// ---------------------------------------------------------------- harness tables (never read by the algorithm logic)
inline std::map<bID, int> regionOf;
inline std::map<int, int> rootCellOf;                    // gpsr baseline: the location an anycast is addressed to (the region root)
inline std::map<int, std::vector<bID>> members;          // region -> module ids (sorted)
inline std::map<bID, int> cellOf;                        // module -> grid cell index

// traffic: one packet per (source, other region); dates spread robot-wide
inline std::vector<std::pair<Time, int>> trafficPlan(bID id, int myRegion) {
    std::vector<std::pair<Time, int>> plan;
    if (cfg.traffic == "geocast") {                       // a few modules each send one geocast to another region
        if (cfg.traceDst > 0) { if (myRegion != cfg.traceDst) plan.push_back({cfg.start, cfg.traceDst}); return plan; }   // GUI: one geocast
        if (id % cfg.geoEvery != 0) return plan;
        std::vector<int> others; for (int r = 1; r <= NB_REGIONS; r++) if (r != myRegion) others.push_back(r);
        plan.push_back({cfg.start + (Time)id * cfg.spacing, others[(id / cfg.geoEvery) % others.size()]});
        return plan;
    }
    if (cfg.traffic == "hotspot") {                       // a stream from the source regions to ONE region (anycast)
        if (cfg.hotSrc.find(char('0' + myRegion)) == std::string::npos || myRegion == cfg.hotDst) return plan;
        Time off = (Time)((id * 7919u) % 1000u) * cfg.hotPeriod / 1000;
        for (int k = 0; k < cfg.hotCount; k++) plan.push_back({cfg.start + off + (Time)k * cfg.hotPeriod, cfg.hotDst});
        return plan;
    }
    int j = 0;
    for (int r = 1; r <= NB_REGIONS; r++) {
        if (r == myRegion) continue;
        if (cfg.traceDst > 0 && r != cfg.traceDst) continue;
        Time t = cfg.spacing == 0 ? cfg.start : cfg.start + (Time)((id - 1) * (NB_REGIONS - 1) + j) * cfg.spacing;
        plan.push_back({t, r});
        j++;
    }
    return plan;
}
// unicast target for (source, region): the application decides which cube it talks to
inline bID targetFor(bID src, int r) {
    auto &m = members[r];
    if (m.empty()) return 0;
    if (cfg.targets == "hot") {
        int n = std::min<int>(cfg.hotN, (int)m.size());
        return m[(m.size() * ((src * 7919u) % n)) / n + 0];   // n cubes spread over the region
    }
    return m[(src * 7919u + r * 104729u) % m.size()];
}

// ---------------------------------------------------------------- failures
static const int KILL_EVENT = -1, HB_EVENT = -2, MAP_EVENT = -3, STATE_EVENT = -4, PARK_EVENT = -5, SPLIT_EVENT = -6, SUBHOLD_EVENT = -7, SEEK_EVENT = -8, RETX_EVENT = -9, WIDEN_EVENT = -50, MDELFLUSH_EVENT = -51, HOLD_EVENT = -10;   // HOLD_EVENT - r
inline std::set<bID> dead;
inline std::map<bID, Time> deathTime;                    // when each module died (failure or empty battery)
inline std::vector<bID> killed;
inline bool isAlive(BuildingBlock *b) { return b->getState() >= BuildingBlock::ALIVE && !dead.count(b->blockId); }

inline std::ofstream *evlog = nullptr;
inline bool evlogTried = false;
inline void logEv(const char *ev, bID id, int pid = -1, int value = -1) {
    if (!evlogTried) { evlogTried = true; if (const char *f = std::getenv("RR_LOG")) { evlog = new std::ofstream(f); *evlog << "t,ev,id,pid,v\n"; } }
    if (evlog) *evlog << BaseSimulator::getScheduler()->now() << "," << ev << "," << id << "," << pid << "," << value << "\n";
}
// silent: the cube just stops (handlers check isAlive); nobody is told.
// disconnect: VisibleSim unplugs it and tells the neighbours (the old baseline).
inline void kill(BuildingBlock *b) {
    if (!isAlive(b)) return;
    if (cfg.failMode == "disconnect") { BaseSimulator::getWorld()->disconnectBlock(b, false); b->setState(BuildingBlock::REMOVED); }
    dead.insert(b->blockId);
    deathTime[b->blockId] = BaseSimulator::getScheduler()->now();
    killed.push_back(b->blockId);
    logEv("kill", b->blockId);
}

// ---------------------------------------------------------------- statistics
enum Kind { K_ADV, K_SUB, K_HB, K_MAP, K_MAPDEL, K_DV, K_STATE, K_SPLIT, K_DATA, K_ACK, K_N };
[[maybe_unused]] static const char *KIND[K_N] = {"adv", "sub", "hb", "map", "mapdel", "dv", "state", "split", "data", "ack"};
struct Counters {
    long msgs[K_N] = {0}, bytes[K_N] = {0};
    long afterFail[K_N] = {0};                 // messages sent after the failure date
    long drops = 0, lostAtDead = 0;
    long plans = 0, planWork = 0, boxSteps = 0, routeSteps = 0, cacheHits = 0, reroutes = 0;
    long paint = 0, expand = 0, boxTests = 0, geoSteps = 0, recoveries = 0, subPiggy = 0;
    long geoChecks = 0, geoChecksOk = 0, analysisWork = 0, regionGeo = 0;
    long poisons = 0, holdWakes = 0, impossibleFast = 0, marked = 0, waves = 0, faceChecks = 0, perimSteps = 0;
    long detours = 0, switches = 0, awareChoices = 0, energyDeaths = 0, byes = 0;
    long hbEarly = 0, hbEarlyAll = 0, hbSkipped = 0, blindDetours = 0;
    long advReq = 0, advResync = 0, salvaged = 0, parked = 0, parkExpired = 0, parkFull = 0, logFull = 0, dupDiscards = 0, dupDeliveries = 0, suspectAvoid = 0, resentMsgs = 0, dropInreg = 0, dropHops = 0, inregParked = 0, escapes = 0, escapeHops = 0, escapeDead = 0, escapeGiveUp = 0, escapeOk = 0, escapeOkHops = 0, e2eResent = 0, e2eGiveUp = 0, e2eAcked = 0, e2eGone = 0, e2eNoway = 0, widens = 0, widenMax = 0, widenDead = 0, hbSlowSkipped = 0, deadEndDrops = 0, linkReleased = 0, unreachSent = 0, unreachStop = 0, linkRetx = 0, linkDup = 0, linkReports = 0, mapDelBatches = 0, acksSent = 0, nacksSent = 0, e2eLate = 0, e2ePeak = 0, pieceCuts = 0, pieceExits = 0, linkLostCtl = 0, linkLostData = 0, walks = 0, quietParked = 0, splitRenamed = 0, walkFallbacks = 0, walkFound = 0, walkFail = 0, walkHops = 0, gwAnnounce = 0, walkRepair = 0, splitRedirects = 0, splitFanout = 0, subKeys = 0, seeks = 0, seekReplies = 0, seekGiveUp = 0, splitParked = 0; Time firstDeath = 0; double maxLinkU = 0;
    std::map<bID, long> dataTx;                // data messages sent per module (load spread)
    long detections = 0, falsePos = 0, resurrect = 0; double detectDelaySum = 0; Time detectMax = 0;
    Time lastAdv = 0, lastInreg = 0;
    std::map<bID, long> txPerModule;
};
inline Counters cnt;
inline void count(Kind k, bID from, unsigned bytes) {
    Time now = BaseSimulator::getScheduler()->now();
    cnt.msgs[k]++; cnt.bytes[k] += bytes; cnt.txPerModule[from]++;
    if (cfg.failAt > 0 && now >= cfg.failAt) cnt.afterFail[k]++;
    if (k == K_ADV && now > cnt.lastAdv) cnt.lastAdv = now;
    if ((k == K_MAP || k == K_DV) && now > cnt.lastInreg) cnt.lastInreg = now;
    if (k == K_ADV || k == K_MAP || k == K_DV || k == K_MAPDEL) logEv("ctl", from);
}

struct PacketRecord {
    bID src = 0; int srcRegion = 0, dstRegion = 0; bID dst = 0;
    Time sent = 0, delivered = 0; bool ok = false; int hops = -1, inHops = -1; bID by = 0; long tx = 0; int retx = 0; bool rel = false;
};
inline std::vector<PacketRecord> packets;
inline std::vector<PacketRecord> acks;                    // e2e acknowledgements (kept apart: not packets of the application)
static const int ACKBASE = 1 << 24;
static const int FANBASE = 1 << 28;                      // keys of geocast piece copies waiting for their receipt
inline bool isAckPid(int pid) { return pid >= ACKBASE; }
inline PacketRecord &rec(int pid) { return isAckPid(pid) ? acks[pid - ACKBASE] : packets[pid]; }
inline int newAck(bID src, int sr, int dr, bID dst, Time now) {
    PacketRecord r; r.src = src; r.srcRegion = sr; r.dstRegion = dr; r.dst = dst; r.sent = now;
    acks.push_back(r); return ACKBASE + (int)acks.size() - 1;
}
inline std::map<int, std::set<bID>> geoWho;             // geocast: which modules got it
inline std::map<int, std::pair<long, Time>> geoRecv;     // geocast: modules reached, time of the last one
inline int newPacket(bID src, int sr, int dr, bID dst, Time now) {
    PacketRecord r; r.src = src; r.srcRegion = sr; r.dstRegion = dr; r.dst = dst; r.sent = now;
    packets.push_back(r); return (int)packets.size() - 1;
}
inline void countData(bID from, int pid, unsigned bytes) { if (isAckPid(pid)) { count(K_ACK, from, bytes); acks[pid - ACKBASE].tx++; return; } count(K_DATA, from, bytes); packets[pid].tx++; cnt.dataTx[from]++; logEv("tx", from, pid); }
inline void noteDrop(bID at, int pid) { cnt.drops++; logEv("drop", at, pid); }
inline bool deliver(int pid, bID by, int hops, int inHops, Time now) {
    PacketRecord &r = rec(pid);
    if (r.ok) return false;
    r.ok = true; r.hops = hops; r.inHops = inHops; r.by = by; r.delivered = now;
    logEv("deliver", by, pid, hops);
    return true;
}

// ---------------------------------------------------------------- oracle (end of run, alive graph)
inline std::map<bID, int> bfsFrom(const std::vector<bID> &srcs, int onlyRegion = 0) {
    std::map<bID, int> d; std::deque<bID> q;
    for (bID s : srcs) { d[s] = 0; q.push_back(s); }
    while (!q.empty()) {
        bID u = q.front(); q.pop_front();
        BuildingBlock *bb = BaseSimulator::getWorld()->getBlockById(u);
        for (auto *itf : bb->getP2PNetworkInterfaces()) {
            if (!itf->isConnected()) continue;
            bID v = itf->getConnectedBlockId();
            BuildingBlock *vb = BaseSimulator::getWorld()->getBlockById(v);
            if (!isAlive(vb) || d.count(v)) continue;
            if (onlyRegion && regionOf[v] != onlyRegion) continue;
            d[v] = d[u] + 1; q.push_back(v);
        }
    }
    return d;
}

} // namespace r2

#endif
