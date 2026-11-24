#include <random>
/**
 * @file bbRDRM5Code.cpp
 * RDRM v5 - see bbRDRM5Code.hpp. v5 = v4 + recovery buffer (salvage) + optional suspect-after-1-miss.
 *
 * author: Nawar ELZEIN
 **/

#include "bbRDRM5Code.hpp"

using namespace r2;

static int DMAX = 255;
static const Color VISITED(90, 90, 90);
static const Color DELIVERED(255, 255, 255);
static bool reported = false;
static bool geoWalk(const rmap::Bytes &map, rmap::C3 cur, const rmap::C3 &tgt, rmap::Work &w);
// Is the whole rectangle spanned by a and b inside the region? (the boxes never overlap: compare areas)
// If yes, every step that brings a packet closer stays inside: going straight is safe and shortest.
static bool hullCovered(const rmap::Bytes &map, const rmap::C3 &a, const rmap::C3 &b, long &ops) {
    int lo[3], hi[3]; long area = 1, cov = 0;
    for (int i = 0; i < 3; i++) { lo[i] = std::min(a[i], b[i]); hi[i] = std::max(a[i], b[i]); area *= hi[i] - lo[i] + 1; }
    for (int k = 0; k < rmap::boxCount(map); k++) {
        ops++;
        rmap::Box x = rmap::boxAt(map, k); rmap::C3 l = rmap::cell(x.lo), h = rmap::cell(x.hi);
        long v = 1;
        for (int i = 0; i < 3; i++) { int s = std::max(lo[i], l[i]), e = std::min(hi[i], h[i]); if (e < s) { v = 0; break; } v *= e - s + 1; }
        cov += v;
    }
    return cov == area;
}


static inline int bit(int f) { return 1 << f; }
static inline int popc(int m) { return __builtin_popcount(m); }
static const double US_PER_BYTE = 6095.0 / 17;        // VisibleSim Blinky Blocks link: 17 B in ~6.1 ms
static inline double hinge(double x, double t) { return x <= t ? 0 : ((x - t) / (1 - t)) * ((x - t) / (1 - t)); }

BbRDRM5Code::BbRDRM5Code(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host), module(host) {
    for (int r = 0; r <= NB_REGIONS; r++) { dist[r] = INF; holdUntil[r] = 0; for (int f = 0; f < MAXF; f++) nbDist[f][r] = INF; }
    for (int f = 0; f < MAXF; f++) { faceCell[f] = -1; lastHeard[f] = 0; lastSent[f] = 0; faceRegion[f] = -1;
                                     txUs[f] = 0; linkU[f] = 0; nbLoad[f] = 0; nbEnergy[f] = 0; nbAhead[f] = 0; }
    for (int r = 0; r <= NB_REGIONS; r++) { sticky[r] = -1; rrCount[r] = 0; }
    for (int f = 0; f < MAXF; f++) { toldLoad[f] = toldEnergy[f] = toldAhead[f] = 0; lastDataFrom[f] = 0; known[f] = false; }
    if (!host) return;
    for (int id : {ADV_ID, SUB_ID, HB_ID, MTREE_ID, MECHO_ID, MDOWN_ID, MDEL_ID, DV_ID, STATE_ID, BYE_ID, UDATA_ID, SUBADV_ID, RREQ_ID, RREP_ID, WALK_ID, WALKREP_ID, GW_ID, LIM_ID, MADD_ID})
        addMessageEventFunc2(id, std::bind(&BbRDRM5Code::onMsg, this, std::placeholders::_1, std::placeholders::_2));
}

void BbRDRM5Code::parseUserElements(TiXmlDocument *config) {
    parseExperiment(config);
    DMAX = cfg.dmax;
}

void BbRDRM5Code::parseUserBlockElements(TiXmlElement *config) {
    if (const char *a = config->Attribute("region")) region = std::stoi(a);
    if (const char *a = config->Attribute("source")) isSource = Simulator::extractBoolFromString(a);
    if (const char *a = config->Attribute("fail")) willFail = Simulator::extractBoolFromString(a);
    if (const char *a = config->Attribute("failTime")) myFailAt = std::stoull(a);   // test: this module dies at its own date
    if (const char *a = config->Attribute("root")) isLeader = isRootCfg = Simulator::extractBoolFromString(a);
    regionOf[module->blockId] = region;
}

static double lossDraw() { static std::mt19937 g(cfg.lossSeed); static std::uniform_real_distribution<double> u(0, 1); return u(g); }
template <class M> void BbRDRM5Code::sendCtl(int f, M *msg, Kind k) {
    if (!usable(f)) { delete msg; return; }
    unsigned sz = msg->size();
    if (cfg.linkLoss > 0 && getScheduler()->now() >= cfg.lossFrom && lossDraw() < cfg.linkLoss) { cnt.linkLostCtl++; delete msg; }   // lost on the wire
    else sendMessage(msg, module->getInterface(f), 0, 0);
    count(k, module->blockId, sz);
    txUs[f] += sz * US_PER_BYTE; spend(sz * cfg.eTx);
    lastSent[f] = getScheduler()->now();
}

bool BbRDRM5Code::usable(int f) const {
    return f >= 0 && f < module->getNbInterfaces() && f < MAXF && module->getInterface(f)->isConnected() && !(deadMask & bit(f));
}
int BbRDRM5Code::inRegionMask() const {
    int m = 0;
    for (int f = 0; f < MAXF; f++) if (usable(f) && nbDist[f][region] == 0) m |= bit(f);
    return m;
}

void BbRDRM5Code::startup() {
    rmap::G() = rmap::C3{(int)BaseSimulator::getWorld()->lattice->gridSize[0], (int)BaseSimulator::getWorld()->lattice->gridSize[1],
                         (int)BaseSimulator::getWorld()->lattice->gridSize[2]};
    battery = cfg.battery;                               // before any message is sent
    if (cfg.lowPct > 0 && (int)(((module->blockId * 2654435761u) ^ (cfg.lowSeed * 40503u)) % 10000u) < (int)(cfg.lowPct * 100))
        battery = cfg.battery * cfg.lowLevel;            // a weak module (older battery)
    myCell = rmap::C3{module->position[0], module->position[1], module->position[2]};
    myIdx = rmap::idx(myCell);
    cellOf[module->blockId] = myIdx;
    {   // harness tables (targets of the traffic plan, statistics)
        auto &m = members[region];
        m.insert(std::upper_bound(m.begin(), m.end(), module->blockId), module->blockId);
    }
    for (int f = 0; f < module->getNbInterfaces() && f < MAXF; f++) {
        P2PNetworkInterface *itf = module->getInterface(f);
        if (!itf->isConnected()) continue;
        const Cell3DPosition &p = BaseSimulator::getWorld()->getBlockById(itf->getConnectedBlockId())->position;
        faceCell[f] = rmap::idx(rmap::C3{p[0], p[1], p[2]});   // = my cell + the face direction
    }
    if (cfg.stopAt == 0) {
        long n = BaseSimulator::getWorld()->getMap().size();
        cfg.stopAt = (cfg.traceDst > 0) ? cfg.start + 4000000
                     : cfg.traffic == "hotspot" ? cfg.start + (Time)cfg.hotCount * cfg.hotPeriod + 3000000
                     : cfg.start + (Time)(n * (NB_REGIONS - 1)) * cfg.spacing + 3000000;
        cfg.stopAt += cfg.tail;
    }
    if (cfg.allSources) isSource = true;
    if (cfg.leader == "elect") isLeader = false;         // no leader in the config: the region elects one
    for (int f = 0; f < MAXF; f++) lastHeard[f] = 0;
    dist[region] = (cfg.baseline == "lengo" && !isRootCfg) ? INF : 0;   // Le & Ngo: only the root is a source
    if (isRootCfg) rootCellOf[region] = myIdx;
    if (mergedEcho()) {            // the region's own ADV(region, 0) IS the map wave: only the leader starts it
        if (isLeader) { joined = true; echoStarted = true; echoParent = -1; subtree = rmap::single(myCell); announce(region); }
    } else announce(region);
    if (willFail && (myFailAt > 0 || cfg.failAt > 0))
        getScheduler()->schedule(new InterruptionEvent<int>(myFailAt > 0 ? myFailAt : cfg.failAt, module, KILL_EVENT));
    if (cfg.detect == "hb")
        getScheduler()->schedule(new InterruptionEvent<int>(cfg.hbPeriod / 10 + (module->blockId * 7919) % cfg.hbPeriod, module, HB_EVENT));
    if ((cfg.traffic == "unicast" || cfg.e2e == "on" || (cfg.traffic == "geocast" && cfg.split != "none")) && (cfg.inreg == "dv" || (usesMap() && !mergedEcho())))   // e2e: acknowledgements go to one module: the maps are needed for anycast too   // geocast + split: the map tells the pieces apart
        getScheduler()->schedule(new InterruptionEvent<int>(cfg.mapAt + (cfg.leader == "elect" ? (Time)((module->blockId * 37) % 97) * 5000 : 0), module, MAP_EVENT));
    if (cfg.balance != "none" || cfg.energy == "on" || cfg.traffic == "hotspot")
        getScheduler()->schedule(new InterruptionEvent<int>(cfg.statePeriod + (module->blockId * 7919) % cfg.statePeriod, module, STATE_EVENT));
    if (isSource)
        for (auto &t : trafficPlan(module->blockId, region))
            getScheduler()->schedule(new InterruptionEvent<int>(t.first, module, t.second));
}

// ------------------------------------------------------------------ gradients
void BbRDRM5Code::announce(int r) {
    int d = dist[r] >= DMAX ? DMAX : dist[r];
    for (int f = 0; f < MAXF; f++) {
        if (!usable(f)) continue;
        AdvMsg *a = new AdvMsg(r, advDist(r, f), region);
        {
            if (cfg.mergeSub && ((subMask ^ sentSub) & bit(f))) {                     // the subscription rides on this ADV
                a->flags |= (subMask & bit(f)) ? 2 : 4;
                sentSub = (sentSub & ~bit(f)) | (subMask & bit(f)); cnt.subPiggy++;
            }
            if (r == region && d == 0 && mergedEcho() && joined && f == echoParent) a->flags |= 1;
        }
        sendCtl(f, a, K_ADV);
    }
}

int BbRDRM5Code::bestFace(int r) const {
    int best = -1, bd = INF; bID bid = 0;
    for (int f = 0; f < MAXF; f++) {
        if (!usable(f) || nbDist[f][r] >= DMAX || nbDist[f][r] >= dist[r]) continue;
        bID id = module->getInterface(f)->getConnectedBlockId();
        if (nbDist[f][r] < bd || (nbDist[f][r] == bd && id < bid)) { best = f; bd = nbDist[f][r]; bid = id; }
    }
    return best;
}

bool BbRDRM5Code::recalc(int r) {
    int d;
    if (r == region && (cfg.baseline != "lengo" || isRootCfg)) d = 0;
    else {
        int m = INF;
        for (int f = 0; f < MAXF; f++) if (usable(f)) m = std::min(m, nbDist[f][r]);
        d = (m >= DMAX - 1) ? INF : m + 1;
        if (cfg.cti == "poison") {
            Time now = getScheduler()->now();
            if (dist[r] < INF && d > dist[r]) {          // bad news: never climb, call it impossible now
                d = INF; holdUntil[r] = now + cfg.hold; cnt.poisons++;
                getScheduler()->schedule(new InterruptionEvent<int>(holdUntil[r], module, HOLD_EVENT - r));
            } else if (dist[r] >= INF && now < holdUntil[r]) d = INF;   // still holding: old values may be stale
        }
    }
    if (d == dist[r]) return false;
    dist[r] = d;
    return true;
}
// recompute every region, then subscriptions, then announce what changed (subscriptions ride along)
void BbRDRM5Code::recomputeAll() {
    bool ch[NB_REGIONS + 1];
    for (int r = 1; r <= NB_REGIONS; r++) ch[r] = recalc(r);
    updateSubs();
    for (int r = 1; r <= NB_REGIONS; r++) if (ch[r]) announce(r);
    flushSubs();
}

// a child subscribes to the faces it forwards through (its parents), one SUB per face
void BbRDRM5Code::updateSubs() {
    if (cfg.detect != "hb") return;
    if (cfg.hbScope == "all") {                          // everyone beats to everyone: no subscription messages
        int want = 0;
        for (int r = 1; r <= NB_REGIONS; r++) { if (r == region || dist[r] >= DMAX) continue; int f = bestFace(r); if (f >= 0) want |= bit(f); }
        subMask = sentSub = want;
        return;
    }
    int want = 0;
    for (int r = 1; r <= NB_REGIONS; r++) {
        if (r == region || dist[r] >= DMAX) continue;
        int f = bestFace(r); if (f >= 0) want |= bit(f);
    }
    Time now = getScheduler()->now();
    for (int f = 0; f < MAXF; f++) if ((want & bit(f)) && !(subMask & bit(f))) lastHeard[f] = now;
    subMask = want;
    if (!cfg.mergeSub) flushSubs();
}
// what could not ride on an ADV goes as a standalone 1-byte SUB
void BbRDRM5Code::flushSubs() {
    if (cfg.detect != "hb") return;
    int diff = subMask ^ sentSub;
    for (int f = 0; f < MAXF; f++) if ((diff & bit(f)) && usable(f)) sendCtl(f, new SubMsg((subMask & bit(f)) != 0), K_SUB);
    sentSub = subMask;
}

// ------------------------------------------------------------------ failure detection
void BbRDRM5Code::heard(int f) {
    lastHeard[f] = getScheduler()->now();
    if (deadMask & bit(f)) {                 // it was only late: take it back
        deadMask &= ~bit(f); cnt.resurrect++;
        if (cfg.falseFix == "on") {
            falseMask |= bit(f);                             // this link drops beats: wait longer on it from now on
            if (faceCell[f] >= 0 && deletedNb.count(faceCell[f])) { deletedNb.erase(faceCell[f]); mapAdd(faceCell[f], -1); }
        }
        for (int r = 1; r <= NB_REGIONS; r++) if (dist[r] < DMAX) sendCtl(f, new AdvMsg(r, advDist(r, f), region), K_ADV);
    }
}

void BbRDRM5Code::declareDead(int f) {
    Time now = getScheduler()->now();
    bID nb = module->getInterface(f)->getConnectedBlockId();
    if (dead.count(nb)) {
        cnt.detections++;
        Time dl = now - deathTime[nb]; cnt.detectDelaySum += dl; cnt.detectMax = std::max(cnt.detectMax, dl);
    } else cnt.falsePos++;
    bool wasInRegion = nbDist[f][region] == 0;
    deadMask |= bit(f); childMask &= ~bit(f); subMask &= ~bit(f); sentSub &= ~bit(f);
    for (int r = 1; r <= NB_REGIONS; r++) nbDist[f][r] = INF;
    for (auto &e : dvNb) e.second[f] = INF;
    recomputeAll();
    if (wasInRegion && usesMap() && mapReady) { if (faceCell[f] >= 0) deletedNb.insert(faceCell[f]); mapDelete(faceCell[f], -1); }
    if (cfg.inreg == "dv") { std::vector<int> cs; for (auto &e : dvDist) cs.push_back(e.first); for (int c : cs) dvRecompute(c); }
    if (!sub.empty()) { std::vector<int> ks; for (auto &e : sub) { e.second.nb[f] = INF; ks.push_back(e.first); } for (int k : ks) subRecalc(k); }
    for (auto it = routes.begin(); it != routes.end();) if (it->second.face == f) it = routes.erase(it); else ++it;
    for (auto it = wroutes.begin(); it != wroutes.end();) if (!it->second.path.empty() && it->second.path[0] == f) it = wroutes.erase(it); else ++it;
    salvageFace(f);
}

// a neighbour I do not depend on went quiet: just mark it (never use it, erase it from the map)
void BbRDRM5Code::markDead(int f) {
    bID nb = module->getInterface(f)->getConnectedBlockId();
    if (dead.count(nb)) { cnt.detections++; Time dl = getScheduler()->now() - deathTime[nb]; cnt.detectDelaySum += dl; cnt.detectMax = std::max(cnt.detectMax, dl); }
    else cnt.falsePos++;
    cnt.marked++;
    bool wasInRegion = nbDist[f][region] == 0;
    deadMask |= bit(f); childMask &= ~bit(f);
    for (int r = 1; r <= NB_REGIONS; r++) nbDist[f][r] = INF;      // cannot change my distance: it was not my parent
    for (auto &e : dvNb) e.second[f] = INF;
    if (wasInRegion && usesMap() && mapReady) { if (faceCell[f] >= 0) deletedNb.insert(faceCell[f]); mapDelete(faceCell[f], -1); }
    if (cfg.inreg == "dv") { std::vector<int> cs; for (auto &e : dvDist) cs.push_back(e.first); for (int c : cs) dvRecompute(c); }
    if (!sub.empty()) { std::vector<int> ks; for (auto &e : sub) { e.second.nb[f] = INF; ks.push_back(e.first); } for (int k : ks) subRecalc(k); }
    for (auto it = routes.begin(); it != routes.end();) if (it->second.face == f) it = routes.erase(it); else ++it;
    for (auto it = wroutes.begin(); it != wroutes.end();) if (!it->second.path.empty() && it->second.path[0] == f) it = wroutes.erase(it); else ++it;
    salvageFace(f);
}

// the simulator's own alert: only used by the detect="event" baseline
void BbRDRM5Code::onNeighborChanged(uint64_t, int action) {
    if (!isAlive(module) || cfg.detect != "event") return;
    if (action != EVENT_REMOVE_NEIGHBOR) return;
    for (int f = 0; f < module->getNbInterfaces() && f < MAXF; f++)
        if (!module->getInterface(f)->isConnected() && faceCell[f] >= 0 && !(deadMask & bit(f))) {
            bool wasInRegion = nbDist[f][region] == 0;
            deadMask |= bit(f);
            cnt.detections++; Time dl = getScheduler()->now() - cfg.failAt; cnt.detectDelaySum += dl; cnt.detectMax = std::max(cnt.detectMax, dl);
            for (int r = 1; r <= NB_REGIONS; r++) nbDist[f][r] = INF;
            for (auto &e : dvNb) e.second[f] = INF;
            if (wasInRegion && usesMap() && mapReady) { if (faceCell[f] >= 0) deletedNb.insert(faceCell[f]); mapDelete(faceCell[f], -1); }
        }
    recomputeAll();
    if (cfg.inreg == "dv") { std::vector<int> cs; for (auto &e : dvDist) cs.push_back(e.first); for (int c : cs) dvRecompute(c); }
}

// ------------------------------------------------------------------ region map (echo wave)
void BbRDRM5Code::echoStart(int parent) {
    echoStarted = true; echoParent = parent;
    subtree = rmap::single(myCell);
    int m = inRegionMask();
    pending = m & ~(parent >= 0 ? bit(parent) : 0);
    for (int f = 0; f < MAXF; f++) if (pending & bit(f)) sendCtl(f, new MapTreeMsg(), K_MAP);
    checkEcho();
}
void BbRDRM5Code::startWave(int w, int parent) {
    cnt.waves += (parent < 0);
    wave = w; echoStarted = true; echoSent = false; echoParent = parent; mapChildren = 0;
    subtree = rmap::single(myCell);
    pending = inRegionMask() & ~(parent >= 0 ? bit(parent) : 0);
    for (int f = 0; f < MAXF; f++) if (pending & bit(f)) sendCtl(f, new MapTreeMsg(w), K_MAP);
    checkEcho();
}
void BbRDRM5Code::checkEcho() {
    if (pending != 0 || echoSent) return;
    echoSent = true;
    if (echoParent < 0) {                                   // leader: the whole region is in
        if (cfg.leader == "elect") isLeader = true;
        installMap(subtree);
        for (int f = 0; f < MAXF; f++) if (mapChildren & bit(f)) sendCtl(f, new MapBoxesMsg(MDOWN_ID, regionMap, -1), K_MAP);
    } else sendCtl(echoParent, new MapBoxesMsg(MECHO_ID, subtree, -1, cfg.leader == "elect" ? wave : -1), K_MAP);                                  // subtree is freed when the map arrives
}
// merged map wave: done when every in-region neighbour (but my parent) has either joined through
// someone else (its ADV(region,0) without the parent flag) or sent me its subtree (ECHO)
void BbRDRM5Code::mergedCheck() {
    if (!mergedEcho() || !joined || echoSent) return;
    int need = 0;
    for (int f = 0; f < MAXF; f++) {
        if (!usable(f)) continue;
        if (faceRegion[f] < 0) return;                          // not heard from yet
        if (faceRegion[f] == region && f != echoParent) need |= bit(f);
    }
    int ok = (doneMask & ~mapChildren) | echoGot;
    if (need & ~ok) return;
    echoSent = true;
    if (echoParent < 0) {
        installMap(subtree);
        for (int f = 0; f < MAXF; f++) if (mapChildren & bit(f)) sendCtl(f, new MapBoxesMsg(MDOWN_ID, regionMap, -1), K_MAP);
    } else sendCtl(echoParent, new MapBoxesMsg(MECHO_ID, subtree), K_MAP);
}
const rmap::BBox &BbRDRM5Code::box() {
    if (bboxVer != mapVersion) { bbox = rmap::bboxOf(regionMap); bboxVer = mapVersion; }
    return bbox;
}
void BbRDRM5Code::installMap(const rmap::Bytes &b, int score) {
    regionMap = b; mapReady = true; mapVersion++;
    if (score >= 0) geoScore = score / 100.0;
    if (cfg.inreg == "adapt") {                  // every cube reads the shape of its region once, from the map:
        long ops = 0; int k = rmap::boxCount(regionMap), ok = 0, n = 0;   // block = straight is safe between any two boxes
        for (int i = 0; i < k; i++) for (int j = i + 1; j < k; j++) {
            rmap::Box a = rmap::boxAt(regionMap, i), b = rmap::boxAt(regionMap, j);
            rmap::C3 al = rmap::cell(a.lo), ah = rmap::cell(a.hi), bl = rmap::cell(b.lo), bh = rmap::cell(b.hi);
            rmap::C3 lo{std::min(al[0], bl[0]), std::min(al[1], bl[1]), std::min(al[2], bl[2])};
            rmap::C3 hi{std::max(ah[0], bh[0]), std::max(ah[1], bh[1]), std::max(ah[2], bh[2])};
            n++; if (hullCovered(regionMap, lo, hi, ops)) ok++;
        }
        geoScore = n ? double(ok) / n : 1.0;        // 1.0 = block: always go straight, no test per packet
        if (isLeader) cnt.analysisWork += ops;
    }
    rmap::Bytes().swap(subtree);
}
void BbRDRM5Code::flushDel() {
    delTimer = false;
    int m = inRegionMask();
    for (int f = 0; f < MAXF; f++) {
        if (!(m & bit(f))) continue;
        std::vector<int> cs; for (auto &p : delPending) if (p.second != f) cs.push_back(p.first);
        if (!cs.empty()) { sendCtl(f, new MapDelMsg(cs), K_MAPDEL); cnt.mapDelBatches++; }
    }
    delPending.clear();
}
void BbRDRM5Code::mapAdd(int c, int fromFace, int ver) {
    if (ver < 0) ver = cellVer[c] + 1;                    // I am the one who heard it again
    if (!mapReady || ver <= cellVer[c]) return;            // old news
    cellVer[c] = ver;
    if (rmap::has(regionMap, rmap::cell(c))) return;
    regionMap = rmap::uniteB(regionMap, rmap::single(rmap::cell(c)), peakScratch); mapVersion++;
    if (cfg.inreg == "adapt") geoScore = 0;
    int m = inRegionMask();
    for (int f = 0; f < MAXF; f++) if ((m & bit(f)) && f != fromFace) sendCtl(f, new MapAddMsg(c, ver), K_MAPDEL);
    computePiece();
    if ((cfg.split == "grad" || cfg.split == "local" || cfg.split == "walk") && !splitTimer) { splitTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(getScheduler()->now() + cfg.splitSettle, module, SPLIT_EVENT)); }
}
void BbRDRM5Code::mapDelete(int c, int fromFace, int ver) {
    if (cfg.falseFix == "on" && cellVer.count(c)) {        // a cell that was re-added: only newer news deletes it
        if (ver == -1) ver = cellVer[c] + 1;               // my own detection: newest news
        else if (ver == -2) ver = 0;                       // an unversioned deletion: older than the re-add
        if (ver <= cellVer[c]) return;
        cellVer[c] = ver;
    } else if (ver > 0) cellVer[c] = ver;
    if (!mapReady || !rmap::has(regionMap, rmap::cell(c))) return;
    cellDelT[c] = getScheduler()->now();
    regionMap = rmap::removeCellB(regionMap, rmap::cell(c), peakScratch); mapVersion++;
    if (cfg.inreg == "adapt") geoScore = 0;             // a new hole: the region is no longer a block, test per packet
    if (cfg.mapDelBatch > 0) {                            // bundle: deaths in a cascade travel together
        delPending.push_back({c, fromFace});
        if (!delTimer) { delTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(getScheduler()->now() + cfg.mapDelBatch, module, MDELFLUSH_EVENT)); }
    } else {
        int m = inRegionMask();
        int v = cellVer.count(c) ? cellVer[c] : 0;
        for (int f = 0; f < MAXF; f++) if ((m & bit(f)) && f != fromFace) sendCtl(f, v ? new MapDelMsg(c, v) : new MapDelMsg(c), K_MAPDEL);
    }
    computePiece();
    if ((cfg.split == "grad" || cfg.split == "local" || cfg.split == "walk") && !splitTimer) { splitTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(getScheduler()->now() + cfg.splitSettle, module, SPLIT_EVENT)); }
}

// ------------------------------------------------------------------ dv inside the region
void BbRDRM5Code::dvRecompute(int c) {
    int cap = 2 * DMAX;
    int d;
    if (c == myIdx) d = 0;
    else {
        int m = INF; auto it = dvNb.find(c);
        if (it != dvNb.end()) { int rm = inRegionMask(); for (int f = 0; f < MAXF; f++) if (rm & bit(f)) m = std::min(m, it->second[f]); }
        d = (m >= cap - 1) ? INF : m + 1;
    }
    auto cur = dvDist.find(c);
    int old = cur == dvDist.end() ? INF : cur->second;
    if (d == old) return;
    if (d >= INF) dvDist[c] = INF; else dvDist[c] = d;
    int rm = inRegionMask(); int w = d >= INF ? cap : d;
    for (int f = 0; f < MAXF; f++) if (rm & bit(f)) sendCtl(f, new DvMsg(c, w), K_DV);
}

// ------------------------------------------------------------------ events
void BbRDRM5Code::onInterruptionEvent(std::shared_ptr<Event> event) {
    int code = static_cast<InterruptionEvent<int> *>(event.get())->data;
    if (code == KILL_EVENT) { setColor(RED); kill(module); return; }
    if (!isAlive(module)) return;
    Time now = getScheduler()->now();
    if (code == HB_EVENT) {
        for (int f = 0; f < MAXF; f++) {
            if (cfg.hbScope == "all" ? !usable(f) : !(childMask & bit(f))) continue;
            if (cfg.linkAck == "on" && repDirty[f]) { sendHb(f, false); continue; }   // a report is waiting: it goes now
            if (cfg.hbAdapt == "on" && !(lastDataFrom[f] > 0 && now - lastDataFrom[f] < hbBusyW())
                && lastSent[f] > 0 && now - lastSent[f] < hbSlowP() - cfg.hbPeriod / 2) { cnt.hbSlowSkipped++; continue; }   // idle link: slow beat
            if (cfg.hbPiggy && lastSent[f] > 0 && now - lastSent[f] < cfg.hbPeriod      // that message was the heartbeat...
                && (cfg.sig != "hb" || (toldLoad[f] == myLoad && toldEnergy[f] == myEnergy && toldAhead[f] == myAhead))) {   // ...and it knows my levels
                cnt.hbSkipped++; continue;
            }
            sendHb(f, false);
        }
        for (int f = 0; f < MAXF; f++) {
            bool watched = cfg.hbScope == "all" ? usable(f) : (subMask & bit(f)) != 0;
            if (!watched) continue;
            Time miss = cfg.hbMiss + ((falseMask & bit(f)) ? cfg.hbMissExtra : 0);
            if (fastWatch(f) ? now - watchFrom(f) <= miss * cfg.hbPeriod : now - lastHeard[f] <= miss * hbSlowP()) continue;
            if (subMask & bit(f)) declareDead(f); else markDead(f);
        }
        if (now + cfg.hbPeriod <= cfg.stopAt)
            getScheduler()->schedule(new InterruptionEvent<int>(now + cfg.hbPeriod, module, HB_EVENT));
        return;
    }
    if (code == SPLIT_EVENT) {                              // my map stopped changing: am I in a piece of a torn region?
        splitTimer = false; pieceInfo();
        int nk = myLow >= 0 ? region * 65536 + myLow : -1;
        if (cfg.split == "walk" || cfg.split == "local" || cfg.split == "grad") {   // the tear is known here now: resend recent geocasts into the new pieces
            if (nPieces > lastPieces && myLow >= 0) {
                while (!gcLog.empty() && now - gcLog.front().first > cfg.gcKeep) gcLog.pop_front();
                for (auto &g : gcLog) for (size_t k = 0; k < plow.size(); k++) if (plow[k] != myLow) {
                    fanOut(g.second, cfg.split == "walk" ? nearestCellOf(plow[k]) : plow[k], (int)k);
                }
            }
            lastPieces = nPieces;
        }
        if ((cfg.split == "grad" || cfg.split == "local") && nk != seedKey) {
            if (seedKey >= 0) { sub[seedKey].seed = false; subRecalc(seedKey); }
            seedKey = nk;
            if (nk >= 0) {
                cnt.pieceCuts++; auto &g = sub[nk]; g.seed = true;
                if (cfg.split == "local") { const rmap::BBox &bb = box(); g.lim = std::max({2, cfg.localPct * (bb.dx() + bb.dy() + bb.dz()) / 100 + cfg.localAdd, regLim[region]}); }
                subRecalc(nk);
                if (cfg.split == "local" && cfg.widen == "always" && !widenTimer) { wantWiden = true; widenTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(now + cfg.widenAfter, module, WIDEN_EVENT)); }
            }
        }
        return;
    }
    if (code == MDELFLUSH_EVENT) { flushDel(); return; }
    if (code == WIDEN_EVENT) {                             // local: can I hear every piece my map shows? If not, the reach was too short
        widenTimer = false;
        pieceInfo();
        if (cfg.split != "local" || nPieces < 2 || myLow < 0 || !sub.count(seedKey) || !wantWiden) return;
        wantWiden = false;                                   // on demand: only while a packet here waits for a piece nobody announced
        bool miss = false;
        for (size_t k = 0; k < plow.size() && !miss; k++) {
            if (plow[k] == myLow) continue;
            bool have = false;
            for (auto &e : sub) if (e.first / 65536 == region && !e.second.seed && e.second.d < INF && pieceOf(e.first % 65536) == plow[k]) { have = true; break; }
            if (!have) { miss = true;
                long alive = 0; const rmap::BBox &bb = box();            // harness only: does that piece still have a live module?
                for (int i = 0; i < bb.n(); i++) if (pcomp[i] == (int)k) { rmap::C3 c = bb.cellOf(i);
                    BuildingBlock *b = BaseSimulator::getWorld()->getBlockByPosition(Cell3DPosition(c[0], c[1], c[2])); if (b && isAlive(b)) alive++; }
                logEv("widen", module->blockId, (int)alive, plow[k]); if (alive == 0) cnt.widenDead++; }
        }
        if (!miss) return;
        int cur = sub[seedKey].lim;
        if (cur == 0 || cur >= DMAX) {                      // already the whole robot: that piece is out of reach (the robot is cut) or dead
            for (size_t k = 0; k < plow.size(); k++) {
                if (plow[k] == myLow) continue;
                bool have = false;
                for (auto &e : sub) if (e.first / 65536 == region && !e.second.seed && e.second.d < INF && pieceOf(e.first % 65536) == plow[k]) { have = true; break; }
                if (!have && ++maxMiss[plow[k]] >= 2) deadEnds.insert(plow[k]);
            }
            widenTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(now + cfg.widenAfter, module, WIDEN_EVENT));
            return;
        }
        widenTo(region, std::min(2 * cur, DMAX), -1);
        cnt.widens++; cnt.widenMax = std::max<long>(cnt.widenMax, std::min(2 * cur, DMAX));
        widenTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(now + cfg.widenAfter, module, WIDEN_EVENT));
        return;
    }
    if (code == SUBHOLD_EVENT) { std::vector<int> ks; for (auto &e : sub) ks.push_back(e.first); for (int k : ks) subRecalc(k); return; }
    if (code == SEEK_EVENT) {                               // a question got no answer in time: ask again, wider
        std::vector<int> late; for (auto &e : seekPending) if (now >= e.second.first) late.push_back(e.first);
        for (int pk : late) {
            int ttl = seekPending[pk].second; seekPending.erase(pk);
            if (cfg.split == "walk") {
                bool have = wroutes.count(pk) || (routes.count(pk) && usable(routes[pk].face));
                int tp = pieceOf(pk % 65536);                       // the question was for a cell: a way to the piece holding it counts
                if (!have && tp >= 0) for (auto &r : wroutes) if (r.first / 65536 == pk / 65536 && pieceOf(r.first % 65536) == tp) have = true;
                if (!have && tp >= 0) for (auto &r : routes) if (usable(r.second.face) && r.first / 65536 == pk / 65536 && pieceOf(r.first % 65536) == tp) have = true;
                if (have) continue;
                SeekInfo si = seekInfo[pk];
                if (si.tries < cfg.walkTries) startWalk(pk, si.cell, si.gc, -1, si.tries + 1);   // try again at once: another aim, a longer walk
                else if (si.tries < 99 && cfg.walkFallback == "seek") {   // walks failed: one flooded question, as far as the target could be by any detour
                    const rmap::BBox &bb = box();
                    int ttl = std::min(2 * DMAX, 2 * rmap::manhattan(myCell, rmap::cell(si.cell)) + bb.dx() + bb.dy() + bb.dz() + 8);
                    cnt.walkFallbacks++; startSeek(pk, si.cell, ttl, si.gc, 99);
                }
                else { cnt.seekGiveUp++; seekFailed[pk] = now; if (si.tries >= 99) floodFailed[pk] = now; }   // nobody answered a flood: the target is gone
                continue;
            }
            bool have = false; for (auto &r : routes) if (usable(r.second.face) && r.first / 65536 == pk / 65536 && pieceOf(r.first % 65536) == pk % 65536) have = true;
            if (have) continue;
            SeekInfo si = seekInfo[pk];
            if (si.tries < cfg.seekTries) startSeek(pk, si.cell, std::min(2 * ttl, 2 * DMAX), si.gc, si.tries + 1);
            else { cnt.seekGiveUp++; seekFailed[pk] = now; }
        }
        return;
    }
    if (code == RETX_EVENT) {                              // e2e: resend what was not acknowledged in time
        retxTimer = false;
        if (now > cfg.stopAt) return;                       // test: the experiment is over
        std::vector<int> due; for (auto &e : pend) if (e.second.due <= now) due.push_back(e.first);
        for (int pid : due) {
            auto it = pend.find(pid); if (it == pend.end()) continue;
            Pend &p = it->second;
            if (p.tries >= cfg.e2eTries) { cnt.e2eGiveUp++; bufB -= p.msg.size() + 4; pend.erase(it); continue; }
            Time gap = curRto(); for (int k = 1; k < p.tries && gap < cfg.rtoMax; k++) gap *= 2;
            p.tries++; p.due = now + std::min(2 * gap, cfg.rtoMax);
            UDataMsg c = p.msg; cnt.e2eResent++; if (pid < FANBASE) packets[pid].retx++;
            if (pid >= FANBASE && mapReady && !rmap::has(regionMap, rmap::cell(c.dstCell))) {   // that piece was renamed: aim at the nearest other piece
                pieceInfo(); int best = -1, bd = 1 << 30;
                for (size_t k = 0; k < plow.size(); k++) if (plow[k] != myLow) { int dd = rmap::manhattan(rmap::cell(plow[k]), rmap::cell(c.dstCell)); if (dd < bd) { bd = dd; best = plow[k]; } }
                if (best < 0) { bufB -= p.msg.size() + 4; pend.erase(it); continue; }   // no other piece any more: done
                c.dstCell = best; p.msg.dstCell = best;
            } logEv("e2eresend", module->blockId, pid);
            route(&c, -1);
            if (!isAlive(module)) return;
        }
        armRetx();
        return;
    }
    if (code == PARK_EVENT) {
        parkTimer = false; flushPark();          // a packet that still has no route parks again, and parkOrDrop re-arms the (single) timer
        return;
    }
    if (code == STATE_EVENT) {
        stateTick();
        if (now + cfg.statePeriod <= cfg.stopAt)
            getScheduler()->schedule(new InterruptionEvent<int>(now + cfg.statePeriod, module, STATE_EVENT));
        return;
    }
    if (code <= HOLD_EVENT) {                              // hold-down over: listen to the neighbours again
        int r = HOLD_EVENT - code; cnt.holdWakes++;
        if (dist[r] >= INF) { bool ch = recalc(r); updateSubs(); if (ch) announce(r); flushSubs(); }
        return;
    }
    if (code == MAP_EVENT && cfg.leader == "elect" && usesMap() && !mergedEcho()) {
        if (wave == (1 << 30)) startWave(module->blockId, -1);   // nobody captured me yet: start my own wave
        return;
    }
    if (code == MAP_EVENT) {
        if (usesMap() && !mergedEcho() && isLeader) echoStart(-1);
        if (cfg.inreg == "dv") dvRecompute(myIdx);
        return;
    }
    for (int r = 1; r <= NB_REGIONS; r++) {                  // traffic
        if (r == region) continue;
        if (code > 0 && r != code) continue;
        if (cfg.traceDst > 0 && r != cfg.traceDst) continue;
        bID dst = cfg.traffic == "unicast" ? targetFor(module->blockId, r) : 0;
        int dc = dst ? cellOf[dst] : -1;
        int pid = newPacket(module->blockId, region, r, dst, now);
        UDataMsg d(pid, module->blockId, region, r, dc);
        if (cfg.balance == "detour") d.det = cfg.detourMax;
        if (cfg.traffic == "geocast") d.gc = 1;
        if (cfg.e2e == "on" && (int)(((unsigned)pid * 2654435761u >> 8) % 100) < cfg.reliablePct) {   // this packet carries the reliable flag
            d.srcCell = myIdx; packets[pid].rel = true;
            pend.emplace(pid, Pend{d, now + curRto(), 1}); bufB += d.size() + 4; peakBufB = std::max(peakBufB, bufB);
            cnt.e2ePeak = std::max<long>(cnt.e2ePeak, (long)pend.size());
            armRetx();
        }
        route(&d, -1);
        if (!isAlive(module)) return;
    }
    if (cfg.traceDst > 0) setColor(VISITED);
}

void BbRDRM5Code::onMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender) {
    if (!isAlive(module)) { if (m->type == UDATA_ID) cnt.lostAtDead++; return; }
    spend(m->size() * cfg.eRx);
    if (!isAlive(module)) { if (m->type == UDATA_ID) cnt.lostAtDead++; return; }
    int f = module->getDirection(sender);
    if (f < 0 || f >= MAXF) return;
    heard(f);
    switch (m->type) {
    case ADV_ID: {
        AdvMsg *a = static_cast<AdvMsg *>(m.get());
        faceRegion[f] = a->src;
        if (a->flags & 8) {                                   // advSync: the neighbour missed some of my adverts: all my distances again
            cnt.advResync++;
            for (int r = 1; r <= NB_REGIONS; r++) sendCtl(f, new AdvMsg(r, advDist(r, f), region), K_ADV);
            break;
        }
        lastAdvIn[f] = getScheduler()->now();
        if (a->flags & 2) childMask |= bit(f);
        if (a->flags & 4) childMask &= ~bit(f);
        bool joinNow = false;
        if (mergedEcho() && a->region == region && a->dist == 0) {      // a neighbour of my region joined the wave
            if (a->flags & 1) mapChildren |= bit(f); else doneMask |= bit(f);
            if (!joined) { joined = true; echoStarted = true; echoParent = f; doneMask &= ~bit(f); subtree = rmap::single(myCell); joinNow = true; }
        }
        nbDist[f][a->region] = a->dist >= DMAX ? INF : a->dist;
        bool ch = recalc(a->region);
        updateSubs();
        if (ch) announce(a->region);
        else if (cutPiece && a->region == region && a->src != region && extDist() != extSent) announce(region);
        if (joinNow) announce(region);
        flushSubs();
        mergedCheck();
        break; }
    case SUB_ID:
        if (static_cast<SubMsg *>(m.get())->on) childMask |= bit(f); else childMask &= ~bit(f);
        break;
    case HB_ID:                                              // heard() did the liveness part
        if (cfg.linkAck == "on" && static_cast<HbMsg *>(m.get())->rep) linkReport(f, static_cast<LinkRep *>(static_cast<HbMsg *>(m.get())));
        if (cfg.advSync == "on" && static_cast<HbMsg *>(m.get())->advSum >= 0 && static_cast<HbMsg *>(m.get())->advSum != advHash(f, false)) {
            Time now = getScheduler()->now();                 // what it advertises is not what I hold: an advert got lost (or is still on its way)
            if (now - lastAdvIn[f] > 2 * cfg.hbPeriod && now - lastAdvReq[f] > 1000000) {
                lastAdvReq[f] = now; cnt.advReq++; AdvMsg *q = new AdvMsg(0, 0, region); q->flags = 8; sendCtl(f, q, K_ADV);
            }
        }
        if (cfg.sig == "hb") { HbMsg *h = static_cast<HbMsg *>(m.get());
            if (h->levels) { nbLoad[f] = h->load; nbEnergy[f] = h->energy; if (!h->keepAhead) nbAhead[f] = h->ahead; known[f] = true; } }
        break;
    case BYE_ID:                                             // a neighbour leaves: same as 3 missed heartbeats, but now
        if (!(deadMask & bit(f))) { if (subMask & bit(f)) declareDead(f); else markDead(f); }
        break;
    case STATE_ID: {
        StateMsg *st = static_cast<StateMsg *>(m.get());
        nbLoad[f] = st->load; nbEnergy[f] = st->energy; nbAhead[f] = st->ahead;
        break; }
    case MTREE_ID: {
        int w = static_cast<MapTreeMsg *>(m.get())->wave;
        if (cfg.leader == "elect" && w >= 0) {               // echo with extinction: the smallest wave wins
            if (w < wave) startWave(w, f);
            else if (w == wave) { pending &= ~bit(f); checkEcho(); }
            break;                                           // a larger wave dies here
        }
        if (!echoStarted) echoStart(f); else { pending &= ~bit(f); checkEcho(); }
        break; }
    case MECHO_ID:
        if (cfg.leader == "elect" && static_cast<MapBoxesMsg *>(m.get())->wave != wave) break;
        subtree = rmap::uniteB(subtree, static_cast<MapBoxesMsg *>(m.get())->boxes, peakScratch);
        mapChildren |= bit(f);
        if (mergedEcho()) { echoGot |= bit(f); mergedCheck(); } else { pending &= ~bit(f); checkEcho(); }
        break;
    case MDOWN_ID:
        installMap(static_cast<MapBoxesMsg *>(m.get())->boxes, static_cast<MapBoxesMsg *>(m.get())->score);
        for (int g = 0; g < MAXF; g++) if (mapChildren & bit(g)) sendCtl(g, new MapBoxesMsg(MDOWN_ID, regionMap, -1), K_MAP);
        break;
    case MADD_ID: mapAdd(static_cast<MapAddMsg *>(m.get())->cell, f, static_cast<MapAddMsg *>(m.get())->ver); break;
    case MDEL_ID: { MapDelMsg *md = static_cast<MapDelMsg *>(m.get()); for (int c : md->cells) mapDelete(c, f, md->ver ? md->ver : -2); break; }
    case SUBADV_ID: {
        SubAdvMsg *a = static_cast<SubAdvMsg *>(m.get());
        bool fresh = !sub.count(a->key);
        auto &g = sub[a->key]; if (fresh) cnt.subKeys++;
        bool grow = a->lim > g.lim && g.lim > 0;           // local: a wider announcement passes even where my distance does not change
        if (a->lim && (g.lim == 0 || a->lim > g.lim)) g.lim = a->lim;
        g.nb[f] = a->dist >= DMAX ? INF : a->dist;
        subRecalc(a->key, grow);
        break; }
    case RREQ_ID: {
        RreqMsg *q = static_cast<RreqMsg *>(m.get());
        long id = ((long)q->origin << 8) | (q->seq & 255);
        Time now = getScheduler()->now();
        auto it = rreqRev.find(id);
        if (it != rreqRev.end() && now - it->second.second < 3000000) break;      // already seen this question
        rreqRev[id] = {f, now};
        if (q->region == region) {                         // the piece holding the cell answers
            pieceInfo();
            bool mine = false;
            if (nPieces >= 2 && myLow >= 0) {
                int p = pieceOf(q->cell);
                if (p == myLow) mine = true;
                else if (q->gc && p < 0) {                  // geocast: a cell next to my piece will do (it may have died)
                    rmap::C3 c = rmap::cell(q->cell);
                    for (int a = 0; a < 3 && !mine; a++) for (int sg = -1; sg <= 1; sg += 2) { rmap::C3 n = c; n[a] += sg; if (rmap::inGrid(n) && pieceOf(rmap::idx(n)) == myLow) mine = true; }
                }
            }
            if (q->cell == myIdx) mine = true;                  // the target itself: never wrong about holding its own cell
            if (mine) { cnt.seekReplies++; sendCtl(f, new RrepMsg(region * 65536 + (myLow >= 0 ? myLow : myIdx), q->origin, q->seq, 0), K_SPLIT); break; }
            if (nPieces >= 2) {                             // a member of another piece that already knows the way
                int tp = pieceOf(q->cell);
                for (auto &r : routes) if (usable(r.second.face) && r.first / 65536 == region && tp >= 0 && pieceOf(r.first % 65536) == tp) {
                    sendCtl(f, new RrepMsg(r.first, q->origin, q->seq, r.second.hops), K_SPLIT); goto rreqDone; }
            }
        }
        if (q->ttl > 1) for (int g = 0; g < MAXF; g++) if (g != f && usable(g)) sendCtl(g, new RreqMsg(q->region, q->cell, q->origin, q->seq, q->ttl - 1, q->gc), K_SPLIT);
        rreqDone:
        break; }
    case WALK_ID: { WalkMsg w = *static_cast<WalkMsg *>(m.get()); w.back.push_back(f); walkStep(&w, f); break; }
    case WALKREP_ID: {
        WalkRepMsg *a = static_cast<WalkRepMsg *>(m.get());
        logEv("walkrep", module->blockId, a->origin, (int)a->back.size());
        if (a->back.empty()) {                              // I asked: I become my piece's gateway to that piece
            if (a->origin != module->blockId) break;
            auto ex = wroutes.find(a->key);
            if (ex != wroutes.end() && !ex->second.path.empty() && ex->second.path.size() <= a->out.size()) break;   // the other walker was quicker or shorter
            bool first = ex == wroutes.end() || ex->second.path.empty();
            cnt.walkFound++;
            auto &w = wroutes[a->key]; w.path = a->out; w.gw = myIdx;
            if (a->want >= 0 && a->want != a->key) { auto &w2 = wroutes[a->want]; w2.path = a->out; w2.gw = myIdx; }   // also under the (older) name asked for
            if (first && region == a->key / 65536) { cnt.gwAnnounce++; for (int g = 0; g < MAXF; g++) if (usable(g) && nbDist[g][region] == 0) sendCtl(g, new GwMsg(a->key, myIdx), K_SPLIT); }
            break;
        }
        WalkRepMsg *r = new WalkRepMsg(*a); int g = r->back.back(); r->back.pop_back();
        sendCtl(g, r, K_SPLIT);
        break; }
    case LIM_ID: {
        LimMsg *a = static_cast<LimMsg *>(m.get());
        if (a->lim > regLim[a->region]) { regLim[a->region] = a->lim; widenTo(a->region, a->lim, f); if (a->ttl > 1) for (int g = 0; g < MAXF; g++) if (g != f && usable(g)) sendCtl(g, new LimMsg(a->region, a->lim, a->ttl - 1), K_SPLIT); }
        break; }
    case GW_ID: {
        GwMsg *a = static_cast<GwMsg *>(m.get());
        auto it = wroutes.find(a->key);
        if (it != wroutes.end() && (it->second.gw <= a->gw || !it->second.path.empty())) break;   // known, or a lower gateway already (one gateway per piece: the lowest cell), or I am one
        wroutes[a->key].gw = a->gw;
        for (int g = 0; g < MAXF; g++) if (g != f && usable(g) && nbDist[g][region] == 0) sendCtl(g, new GwMsg(a->key, a->gw), K_SPLIT);
        break; }
    case RREP_ID: {
        RrepMsg *a = static_cast<RrepMsg *>(m.get());
        auto r = routes.find(a->key);
        if (r == routes.end() || !usable(r->second.face) || a->hops + 1 < r->second.hops) routes[a->key] = {f, a->hops + 1};
        if (a->origin == module->blockId) break;           // the one who asked: its waiting packets leave at the next retry
        long id = ((long)a->origin << 8) | (a->seq & 255);
        auto it = rreqRev.find(id);
        if (it != rreqRev.end() && it->second.first >= 0) sendCtl(it->second.first, new RrepMsg(a->key, a->origin, a->seq, a->hops + 1), K_SPLIT);
        break; }
    case DV_ID: {
        DvMsg *v = static_cast<DvMsg *>(m.get());
        auto it = dvNb.find(v->cell);
        if (it == dvNb.end()) { std::array<int, MAXF> a; a.fill(INF); it = dvNb.emplace(v->cell, a).first; }
        it->second[f] = v->dist >= 2 * DMAX ? INF : v->dist;
        dvRecompute(v->cell);
        break; }
    case UDATA_ID: {
        UDataMsg d(*static_cast<UDataMsg *>(m.get()));
        lastDataFrom[f] = getScheduler()->now();             // this neighbour uses me: it gets my level changes at once
        if (cfg.linkAck == "on" && d.rep) linkReport(f, &d);
        d.rep = false;
        if (cfg.linkAck == "on" && d.lseq >= 0) {           // link layer: drop a copy already received on this link, note the new one
            int s = d.lseq; repDirty[f] = true;
            if (s <= rHi[f] || rSet[f].count(s)) { cnt.linkDup++; break; }
            rSet[f].insert(s); while (rSet[f].count(rHi[f] + 1)) { rSet[f].erase(rHi[f] + 1); rHi[f]++; }
            while ((int)rSet[f].size() > 32) {              // a gap 32 messages old will not be filled: stop waiting for it (nothing to pass on there)
                int nb = *rSet[f].begin();
                for (int q = rHi[f] + 1; q < nb; q++) hSet[f].insert(q);
                rHi[f] = nb - 1;
                while (rSet[f].count(rHi[f] + 1)) { rSet[f].erase(rHi[f] + 1); rHi[f]++; }
            }
            while (hSet[f].count(hBase[f])) { hSet[f].erase(hBase[f]); hBase[f]++; }
            d.inFace = f; d.inSeq = s;
        }
        d.lseq = -1; d.parked = 0; d.lrecv = 0; d.lres = 0;
        d.hops++;
        logEv("visit", module->blockId, d.pid, d.hops);
        if (cfg.traceDst > 0) setColor(VISITED);
        route(&d, f);
        if (!d.parked) linkHandled(d);                      // forwarded, delivered or dropped: the sender's copy is no longer needed
        break; }
    }
}

// ------------------------------------------------------------------ data
int BbRDRM5Code::faceFor(int c) const {
    for (int f = 0; f < MAXF; f++) if (faceCell[f] == c && usable(f)) return f;
    return -1;
}
void BbRDRM5Code::sendData(UDataMsg *d, int f) {
    if (cfg.quietPark == "on" && cfg.salvage == "on" && suspected(f) && !(d->gc && region == d->dstRegion)) {   // option: never into a quiet neighbour (hurts under overload: busy links delay heartbeats)
        if (!d->sr.empty() && d->sri > 0) d->sri--;                  // (the gateway's way: this hop is not taken yet)
        d->swait = 2; cnt.quietParked++; parkOrDrop(d); return;      // longer wait: the repair after "dead" can take ~1.5 s more
    }
    bool logged = cfg.salvage == "on" && cfg.copyLog == "on" && !(d->gc && region == d->dstRegion) && !(d->ack >= 0 && cfg.ackCopy == "off");
    if (cfg.linkAck == "on" && logged) d->lseq = ++txSeq[f];   // only copies kept in the log are numbered (the others cannot be resent)
    UDataMsg *out = new UDataMsg(*d); out->rep = false;
    if (cfg.linkAck == "on" && repDirty[f]) fillRep(f, out);   // the report rides on this data message
    unsigned sz = out->size();
    if (cfg.linkLoss > 0 && getScheduler()->now() >= cfg.lossFrom && lossDraw() < cfg.linkLoss) { cnt.linkLostData++; logEv("wireloss", module->blockId, d->pid); delete out; }
    else sendMessage(out, module->getInterface(f), cfg.hopDelay, 0);
    countData(module->blockId, d->pid, sz);
    txUs[f] += sz * US_PER_BYTE; spend(sz * cfg.eTx);
    { Time nw = getScheduler()->now();
      if (cfg.hbAdapt == "on" && !fastWatch(f)) fastSince[f] = nw;   // first data on an idle link: start expecting fast beats now
      lastDataOut[f] = nw; }
    lastSent[f] = getScheduler()->now();
    if (d->salv) cnt.resentMsgs++;
    if (cfg.salvage == "on" && cfg.copyLog == "on" && !(d->gc && region == d->dstRegion) && !(d->ack >= 0 && cfg.ackCopy == "off")) logSent(f, *d);
    d->lseq = -1;
}

void BbRDRM5Code::fanOut(UDataMsg &g, int targetCell, int k) {
    UDataMsg c = g; c.fan = 1; c.sub = -1; c.dstCell = targetCell; c.wp.clear(); c.wpi = 0; c.recov = -1; c.sr.clear(); c.via = -1;
    cnt.splitFanout++;
    if (cfg.e2e == "on" && cfg.gcAck == "on") {            // this copy waits for the piece's receipt
        int key = FANBASE + (c.pid % (1 << 24)) * 8 + (k & 7);
        c.srcCell = myIdx; c.src = module->blockId; c.srcRegion = region; c.ackKey = key;
        if (!pend.count(key)) { pend.emplace(key, Pend{c, getScheduler()->now() + curRto(), 1}); bufB += c.size() + 4; armRetx(); }
    }
    route(&c, -1);
}
Time BbRDRM5Code::curRto() const {
    if (cfg.rtoAdapt != "on" || srtt < 0) return cfg.rto;
    return std::min(cfg.rtoCap, std::max(cfg.rtoMin, (Time)(srtt + 4 * rttvar)));
}
void BbRDRM5Code::armRetx() {
    if (pend.empty()) return;
    Time m = pend.begin()->second.due; for (auto &e : pend) m = std::min(m, e.second.due);
    Time now = getScheduler()->now(); m = std::max(m, now + 1000);
    if (retxTimer && retxAt <= m) return;
    retxTimer = true; retxAt = m;
    getScheduler()->schedule(new InterruptionEvent<int>(m, module, RETX_EVENT));
}
// e2e: the target (or the target region, when its map says the target is dead) answers the source
void BbRDRM5Code::sendAck(const UDataMsg *d, int gone) {
    if (cfg.e2e != "on" || d->ack >= 0 || d->srcCell < 0) return;
    Time now = getScheduler()->now();
    int apid = newAck(module->blockId, region, d->srcRegion, d->src, now);
    UDataMsg a(apid, module->blockId, region, d->srcRegion, d->srcCell); a.ack = d->ackKey >= 0 ? d->ackKey : d->pid; a.gone = gone;
    if (cfg.balance == "detour") a.det = cfg.detourMax;
    if (gone) cnt.nacksSent++; else cnt.acksSent++;
    bool rp = reparking; reparking = false; route(&a, -1); reparking = rp;
}

void BbRDRM5Code::route(UDataMsg *d, int fromFace) {
    if (d->hops > 4 * DMAX + 200) { cnt.dropHops++; noteDrop(module->blockId, d->pid); return; }
    if (d->ack >= 0 && d->dstCell == myIdx && region == d->dstRegion) {   // e2e: an acknowledgement reached its source
        deliver(d->pid, module->blockId, d->hops, 0, getScheduler()->now());
        auto it = pend.find(d->ack);
        if (it != pend.end() && d->gone == 2) { cnt.e2eNoway++; it->second.due = std::max(it->second.due, getScheduler()->now() + cfg.rtoMax); }   // no way now: resend later, never give up for it
        else if (it != pend.end() && d->gone == 3) {         // "unreachable": stop only if it is still so after rtoMax (a quiet robot has repaired by then)
            Time nw = getScheduler()->now();
            if (it->second.unreachT == 0 || nw - it->second.unreachT < cfg.rtoMax) { if (it->second.unreachT == 0) it->second.unreachT = nw; it->second.due = std::max(it->second.due, nw + cfg.rtoMax); armRetx(); }
            else { cnt.unreachStop++; bufB -= it->second.msg.size() + 4; pend.erase(it); }
        }
        else if (it != pend.end() && d->gone == 1 && it->second.gone < cfg.goneRetries) { it->second.gone++; it->second.due = getScheduler()->now() + cfg.rtoMax; cnt.e2eGone++; armRetx(); }   // maybe a false alarm: ask again later
        else if (it != pend.end()) {
            if (!d->gone && it->second.tries == 1 && it->first < FANBASE) {   // Karn: only receipts of a first try measure the round trip
                double r = (double)(getScheduler()->now() - packets[it->first].sent);
                if (srtt < 0) { srtt = r; rttvar = r / 2; } else { rttvar = 0.75 * rttvar + 0.25 * std::fabs(srtt - r); srtt = 0.875 * srtt + 0.125 * r; }
            }
            if (d->gone) cnt.e2eGone++; else cnt.e2eAcked++; bufB -= it->second.msg.size() + 4; pend.erase(it);
        }
        else cnt.e2eLate++;                                  // a second ack (the packet was sent twice)
        return;
    }
    if (cfg.baseline == "gpsr" && d->ack < 0) {             // baseline: GPSR across the whole robot, no gradient, no recovery
        if (region == d->dstRegion && (d->dstCell < 0 || d->dstCell == myIdx)) {
            if (d->entryHops < 0) d->entryHops = d->hops;
            deliver(d->pid, module->blockId, d->hops, d->hops - d->entryHops, getScheduler()->now()); return;
        }
        if (!gpsrStep(d, fromFace)) { cnt.dropHops++; noteDrop(module->blockId, d->pid); }
        return;
    }
    if (d->esc) {                                                   // escape mode: geographic, any module, until the target cell
        if (region == d->dstRegion && d->dstCell == myIdx) { d->esc = 0; cnt.escapeOk++; cnt.escapeOkHops += d->escLeft; route(d, fromFace); return; }
        escapeStep(d, fromFace); return;
    }
    if (d->via >= 0 && myIdx == d->via) { d->dstCell = d->realCell; d->via = -1; d->wp.clear(); d->wpi = 0; d->how = -1; d->recov = -1; }   // walk: at my piece's gateway
    if (cfg.split != "none") {
        if (d->sub >= 0 && region == d->dstRegion && mapReady) {   // judge arrival by the target cell, not by the piece's name (names change in cascades)
            int t = d->via >= 0 ? d->realCell : d->dstCell;
            if (t >= 0) {
                pieceInfo();
                int tp = myLow >= 0 ? pieceOf(t) : -1;
                if (myLow < 0 ? rmap::has(regionMap, rmap::cell(t)) : tp == myLow) {
                    if (d->via >= 0) d->dstCell = d->realCell;
                    d->sub = -1; d->sr.clear(); d->sri = 0; d->via = -1; d->wp.clear(); d->wpi = 0; d->how = -1; d->recov = -1;
                } else if (tp >= 0 && d->sr.empty()) {          // re-address an old name, only if this module does not know it (never mid-way)
                    bool known = cfg.split == "walk" ? wroutes.count(d->sub) > 0 : (sub.count(d->sub) && sub[d->sub].d < INF);
                    if (!known) { int k = findKey(region, tp, t, d->gc); if (k >= 0 && k != d->sub) d->sub = k; }
                }
            }
        }
        if (d->sub >= 0) {                                   // on its way to one piece of a torn region
            if (inPiece(d->sub)) { if (d->via >= 0) d->dstCell = d->realCell; d->sub = -1; d->sr.clear(); d->sri = 0; d->via = -1; d->wp.clear(); d->wpi = 0; d->how = -1; d->recov = -1; }
            else if (cfg.split == "walk") {
                if (walkForward(d, fromFace)) return;
                d->swait = 1; cnt.splitParked += !reparking; parkOrDrop(d); return;
            } else {
                int f = splitFace(d);
                if (f >= 0) { sendData(d, f); return; }
                auto it = sub.find(d->sub);
                bool gone = region != d->dstRegion && (it == sub.end() || (it->second.d >= INF && getScheduler()->now() >= it->second.hold));
                if (!gone) { if (deadEnd(d)) return; if (region == d->dstRegion) needWiden(); d->swait = 1; cnt.splitParked += !reparking; parkOrDrop(d); return; }
                d->sub = -1; cnt.splitRenamed++;            // that piece's name is gone here: follow the region's gradient, the region re-addresses it
            }
        } else if (region == d->dstRegion && d->dstCell >= 0) {   // unicast (or a geocast copy) entering its region
            pieceInfo();
            int tp = myLow >= 0 ? pieceOf(d->dstCell) : -1;
            if (tp >= 0 && tp != myLow) {                    // the target sits in another piece of my region
                if (d->entryHops < 0) d->entryHops = d->hops;
                int k = findKey(region, tp, d->dstCell, d->gc);
                if (k >= 0) {
                    cnt.splitRedirects++; d->sub = k; d->wp.clear(); d->wpi = 0; d->how = -1; d->recov = -1;
                    if (cfg.split == "walk") { if (walkForward(d, fromFace)) return; d->swait = 1; cnt.splitParked += !reparking; parkOrDrop(d); return; }
                    int f = splitFace(d);
                    if (f >= 0) { sendData(d, f); return; }
                }
                if (deadEnd(d)) return;
                needWiden(); d->swait = 1; cnt.splitParked += !reparking; parkOrDrop(d); return;
            }
        }
    }
    if (region == d->dstRegion && d->gc) {
        if (d->entryHops < 0) { d->entryHops = d->hops; if (!d->fan) sendAck(d, 0); }   // e2e: the region has it
        if (cfg.split != "none" && !d->fan) {                // first module of the region: one copy for every other piece
            d->fan = 1; pieceInfo();
            if (myLow >= 0) for (size_t k = 0; k < plow.size(); k++) if (plow[k] != myLow) {
                fanOut(*d, cfg.split == "walk" ? nearestCellOf(plow[k]) : plow[k], (int)k);
            }
            { Time now = getScheduler()->now(); gcLog.push_back({now, *d}); while (now - gcLog.front().first > cfg.gcKeep) gcLog.pop_front(); }
        }
        if (d->fan && d->dstCell >= 0) { if (d->ackKey >= 0) sendAck(d, 0); d->dstCell = -1; }   // a piece copy: from here it is a plain region flood (gcAck: tell the entry module)
        geocastStep(d, fromFace); return;
    }
    if (region == d->dstRegion) {
        if (d->entryHops < 0) d->entryHops = d->hops;
        if (d->dstCell < 0 || d->dstCell == myIdx) {
            if (cfg.salvage == "on" && d->dstCell >= 0 && cfg.inreg != "flood") {     // the target keeps what it already got: duplicates die here
                if (got.count(d->pid)) { cnt.dupDiscards++; sendAck(d, 0); return; }   // e2e: answer again (the first answer may be lost)
                got.insert(d->pid);
            }
            if (cfg.anyDedup == "on" && d->dstCell < 0 && cfg.inreg != "flood") {   // anycast: a module that already delivered this packet answers again, delivers nothing
                if (got.count(d->pid)) { cnt.dupDiscards++; sendAck(d, 0); return; }
                got.insert(d->pid);
            }
            sendAck(d, 0);
            bool first = deliver(d->pid, module->blockId, d->hops, d->hops - d->entryHops, getScheduler()->now());
            if (!first && cfg.inreg != "flood") cnt.dupDeliveries++;        // anycast: a copy reached another module of the region
            if (first && cfg.traceDst > 0) setColor(DELIVERED);
            if (cfg.inreg == "flood" && d->dstCell >= 0) inRegionStep(d, fromFace);   // the flood goes on anyway
            return;
        }
        if (cfg.e2e == "on" && cfg.nack == "on" && d->ack < 0 && d->dstCell >= 0 && usesMap() && mapReady && !rmap::has(regionMap, rmap::cell(d->dstCell))
            && cfg.goneWait > 0 && cellDelT.count(d->dstCell) && getScheduler()->now() - cellDelT[d->dstCell] < cfg.goneWait) { d->swait = 2; parkOrDrop(d); return; }   // erased a moment ago: maybe a false alarm, wait
        if (cfg.e2e == "on" && cfg.nack == "on" && d->ack < 0 && d->dstCell >= 0 && usesMap() && mapReady && !rmap::has(regionMap, rmap::cell(d->dstCell))) {
            sendAck(d, 1); noteDrop(module->blockId, d->pid); return;   // the region's map says the target is dead: tell the source to stop
        }
        if (!inRegionStep(d, fromFace)) {                 // no way to the target inside the region (map / table not repaired yet)
            if (cutPiece && d->dstCell >= 0 && !d->gc) {      // I sit in a cut-off piece: hand it to the outside, whose gradient leads to the main part
                int best = -1, bd = INF;
                for (int f = 0; f < MAXF; f++) if (usable(f) && faceRegion[f] != region && nbDist[f][region] < bd) { best = f; bd = nbDist[f][region]; }
                if (best >= 0 && bd < DMAX) { cnt.pieceExits++; d->entryHops = -1; d->wp.clear(); d->wpi = 0; d->how = -1; d->recov = -1; sendData(d, best); return; }
            }
            if (cfg.salvage == "on") { if (!reparking) cnt.inregParked++; parkOrDrop(d); }
            else { cnt.dropInreg++; noteDrop(module->blockId, d->pid); }
        }
        return;
    }
    int f = chooseFace(d->dstRegion, d, fromFace);
    if (f < 0) { if (cfg.salvage == "on") parkOrDrop(d); else noteDrop(module->blockId, d->pid); return; }
    sendData(d, f);
}

// ------------------------------------------------------------------ v5: recovery buffer
// A copy of every data packet I send on face f is kept for salvageWin. Nothing is sent while all is well.
// linkAck: the packet that came in on (inFace, inSeq) has been passed on, delivered or dropped here
void BbRDRM5Code::linkHandled(const UDataMsg &d) {
    if (cfg.linkAck != "on" || d.inFace < 0 || d.inSeq < 0) return;
    int f = d.inFace, s = d.inSeq;
    if (s < hBase[f] || hSet[f].count(s)) return;
    hSet[f].insert(s); while (hSet[f].count(hBase[f])) { hSet[f].erase(hBase[f]); hBase[f]++; }
    repDirty[f] = true;
}
// linkAck: a neighbour's report: drop the copies it has passed on; resend over this link the ones it never received
void BbRDRM5Code::fillRep(int f, LinkRep *h) {
    h->rep = true; repDirty[f] = false; cnt.linkReports++;
    h->hBase = hBase[f]; h->hMask = 0; for (int s : hSet[f]) if (s > hBase[f] && s - hBase[f] <= 32) h->hMask |= 1u << (s - hBase[f] - 1);
    h->rHi = rHi[f]; h->rMask = 0; for (int s : rSet[f]) if (s > rHi[f] + 1 && s - rHi[f] - 1 <= 32) h->rMask |= 1u << (s - rHi[f] - 2);
}
void BbRDRM5Code::linkReport(int f, const LinkRep *h) {
    Time now = getScheduler()->now();
    auto passed = [&](int s) { return s < h->hBase || (s > h->hBase && s - h->hBase <= 32 && ((h->hMask >> (s - h->hBase - 1)) & 1u)); };
    auto got = [&](int s) { return s <= h->rHi || (s > h->rHi + 1 && s - h->rHi - 1 <= 32 && ((h->rMask >> (s - h->rHi - 2)) & 1u)); };
    int newest = h->rHi; for (int k = 0; k < 32; k++) if ((h->rMask >> k) & 1u) newest = h->rHi + 2 + k;
    auto &L = sentLog[f];
    std::deque<std::pair<Time, UDataMsg>> keep;
    std::vector<UDataMsg> again;
    for (auto &e : L) {
        int s = e.second.lseq;
        if (s >= 0 && passed(s)) { bufB -= e.second.size() + 2; cnt.linkReleased++; continue; }
        if (s >= 0 && got(s)) e.second.lrecv = 1;
        else if (s >= 0 && e.second.lres < 5 && ((s < newest && now - e.first > cfg.gapWait) || now - e.first > cfg.lossWait)) { e.second.lres++; e.first = now; again.push_back(e.second); }   // a gap, or never confirmed: lost on the link
        keep.push_back(e);
    }
    L.swap(keep);
    for (auto &d : again) { cnt.linkRetx++; linkSend(d, f); }
}
// linkAck: resend a logged copy over the same link, same sequence number (the receiver drops it if it had it)
void BbRDRM5Code::linkSend(UDataMsg &d, int f) {
    if (!usable(f)) return;
    UDataMsg *out = new UDataMsg(d); out->rep = false;
    if (repDirty[f]) fillRep(f, out);
    unsigned sz = out->size();
    if (cfg.linkLoss > 0 && getScheduler()->now() >= cfg.lossFrom && lossDraw() < cfg.linkLoss) { cnt.linkLostData++; delete out; }
    else sendMessage(out, module->getInterface(f), cfg.hopDelay, 0);
    countData(module->blockId, d.pid, sz); txUs[f] += sz * US_PER_BYTE; spend(sz * cfg.eTx); lastSent[f] = getScheduler()->now();
}
void BbRDRM5Code::logSent(int f, const UDataMsg &d) {
    Time now = getScheduler()->now();
    auto &L = sentLog[f];
    while (!L.empty() && L.front().first < now - cfg.salvageWin) { bufB -= L.front().second.size() + 2; L.pop_front(); }
    int cap = (lastHeard[f] > 0 && now - lastHeard[f] > cfg.suspectAfter) ? 4 * cfg.salvageMax : cfg.salvageMax;   // a quiet neighbour may be dead: keep every copy sent to it
    if ((int)L.size() >= cap) { bufB -= L.front().second.size() + 2; L.pop_front(); cnt.logFull++; }
    L.push_back({now, d}); bufB += d.size() + 2;                  // + 2 bytes: the time it was sent
    peakBufB = std::max(peakBufB, bufB);
}
// the neighbour on face f is dead: whatever I sent it recently may have died with it -> route it again
void BbRDRM5Code::salvageFace(int f) {
    if (cfg.salvage != "on") return;
    Time now = getScheduler()->now();
    std::vector<UDataMsg> again;
    for (auto &e : sentLog[f]) { bufB -= e.second.size() + 2; if (e.first >= now - cfg.salvageWin) again.push_back(e.second); }
    sentLog[f].clear();
    for (auto &d : again) {
        cnt.salvaged++; d.salv = 1;
        d.wp.clear(); d.wpi = 0; d.recov = -1;                      // its planned route may cross the dead module: plan again
        d.sr.clear(); d.sri = 0;
        if (d.entryHops >= 0 && region != d.dstRegion) d.entryHops = -1;
        logEv("salvage", module->blockId, d.pid, d.hops);
        route(&d, f);
    }
}
// no route right now (poisoned, hold-down): wait and try again instead of dropping
void BbRDRM5Code::parkOrDrop(UDataMsg *d) {
    Time now = getScheduler()->now(), t = reparking ? curParkT : now;
    if (now - t > (d->swait == 1 ? cfg.splitWait : d->swait == 2 ? cfg.quietWait : cfg.parkTimeout)) {
        // still no way to the target inside its own region: the region is cut in pieces. If my map still lists the target
        // (nobody reported it dead) walk around the cut through the neighbouring regions instead of dropping it.
        if (cfg.escape == "geo" && region == d->dstRegion && d->dstCell >= 0 && !d->gc) {
            if (mapReady && rmap::has(regionMap, rmap::cell(d->dstCell))) { cnt.escapes++; logEv("escape", module->blockId, d->pid, d->dstCell); d->esc = 1; d->recov = -1; d->escLeft = cfg.escapeBudget * rmap::manhattan(myCell, rmap::cell(d->dstCell)) / 100 + 10; escapeStep(d, -1); return; }
            cnt.escapeDead++;
        }
        if (d->ack < 0 && cfg.noway == "on") sendAck(d, 2);
        if (cfg.unreach == "on" && d->ack < 0 && d->swait == 1 && region == d->dstRegion && sub.count(seedKey) && (sub[seedKey].lim == 0 || sub[seedKey].lim >= DMAX)) {
            cnt.unreachSent++; sendAck(d, 3);                // waited for a piece that even a whole-robot announcement did not bring: probably cut off
        }                       // e2e: tell the source there is no way from here (it waits longer before resending)
        cnt.parkExpired++; logEv("expire", module->blockId, d->pid, (mapReady ? 2 : 0) + (mapReady && d->dstCell >= 0 && rmap::has(regionMap, rmap::cell(d->dstCell)) ? 1 : 0)); noteDrop(module->blockId, d->pid); return;
    }
    if ((int)park.size() >= cfg.parkMax) { cnt.parkFull++; noteDrop(module->blockId, park.front().second.pid); linkHandled(park.front().second); park.pop_front(); }
    d->parked = 1;   // full: the one that waited longest leaves (most likely undeliverable)
    if (!reparking) cnt.parked++;
    park.push_back({t, *d}); bufB += d->size() + 2; peakBufB = std::max(peakBufB, bufB);
    if (!parkTimer) { parkTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(now + cfg.parkRetry, module, PARK_EVENT)); }
}
void BbRDRM5Code::flushPark() {
    std::deque<std::pair<Time, UDataMsg>> waiting; waiting.swap(park);
    for (auto &e : waiting) {
        bufB -= e.second.size() + 2;
        e.second.parked = 0;
        curParkT = e.first; reparking = true; route(&e.second, -1); reparking = false;   // parks again (same date) if still no route
        if (!e.second.parked) linkHandled(e.second);
    }
}
// geographic step over the whole robot (greedy, then right-hand rule along the edge of the gap), like GPSR on the grid
void BbRDRM5Code::escapeStep(UDataMsg *d, int fromFace) {
    rmap::C3 tgt = rmap::cell(d->dstCell);
    int md = rmap::manhattan(myCell, tgt);
    cnt.escapeHops++;
    if (--d->escLeft < 0) { cnt.escapeGiveUp++; logEv("giveup", module->blockId, d->pid, d->dstCell); noteDrop(module->blockId, d->pid); return; }   // target dead or out of reach
    if (d->recov >= 0 && md < d->recov) d->recov = -1;
    if (d->recov < 0) {
        int best = -1, bd = md;
        for (int f = 0; f < MAXF; f++) {
            if (!usable(f) || faceCell[f] < 0) continue;
            int dd = rmap::manhattan(rmap::cell(faceCell[f]), tgt);
            if (dd < bd) { best = f; bd = dd; }
        }
        if (best >= 0) { sendData(d, best); return; }
        d->recov = md; fromFace = -1;
    }
    static const int DX[4] = {1, 0, -1, 0}, DZ[4] = {0, -1, 0, 1};
    auto dirOf = [&](int f) { rmap::C3 c = rmap::cell(faceCell[f]); int dx = c[0] - myCell[0], dz = c[2] - myCell[2];
                              for (int k = 0; k < 4; k++) if (DX[k] == dx && DZ[k] == dz) return k; return -1; };
    int start, k0;
    if (fromFace >= 0 && faceCell[fromFace] >= 0 && dirOf(fromFace) >= 0) { start = dirOf(fromFace); k0 = 1; }
    else { int ax = std::abs(tgt[0] - myCell[0]) >= std::abs(tgt[2] - myCell[2]) ? 0 : 2;
           start = ax == 0 ? (tgt[0] > myCell[0] ? 0 : 2) : (tgt[2] > myCell[2] ? 3 : 1); k0 = 0; }
    for (int k = k0; k < k0 + 4; k++) {
        int dir = (start + k) % 4;
        for (int f = 0; f < MAXF; f++) if (usable(f) && faceCell[f] >= 0 && dirOf(f) == dir) { sendData(d, f); return; }
    }
    noteDrop(module->blockId, d->pid);
}
// Which piece of my region am I in? My map knows every death around my piece (its members watch those modules and
// flood the deletions inside the piece), so the parts of the map I can no longer reach are other pieces. The biggest
// part (ties: the one holding the smallest cell) stays "the region"; any other piece stops announcing distance 0 to
// the outside, so packets for the region are led to the main part. Inside the piece nothing changes.
void BbRDRM5Code::computePiece() {
    bool was = cutPiece; cutPiece = false;
    if (cfg.piece == "on" && cfg.salvage == "on" && mapReady) {
        const rmap::BBox &bb = box(); int n = bb.n();
        rmap::Work w; rmap::Bytes v(rmap::bitsBytes(bb), 0); rmap::paintB(v, regionMap, bb, w);
        std::vector<int> comp(n, -1), sz, low; std::vector<int> st;
        for (int i = 0; i < n; i++) {
            if (!rmap::get(v, i) || comp[i] >= 0) continue;
            int k = (int)sz.size(); sz.push_back(0); low.push_back(1 << 30); comp[i] = k; st.push_back(i);
            while (!st.empty()) {
                int u = st.back(); st.pop_back(); sz[k]++;
                rmap::C3 c = bb.cellOf(u); low[k] = std::min(low[k], rmap::idx(c));
                for (int a = 0; a < 3; a++) for (int s = -1; s <= 1; s += 2) {
                    rmap::C3 q = c; q[a] += s;
                    if (!bb.in(q)) continue;
                    int j = bb.li(q);
                    if (rmap::get(v, j) && comp[j] < 0) { comp[j] = k; st.push_back(j); }
                }
            }
        }
        peakScratch = std::max<long>(peakScratch, 4L * n);
        if (sz.size() > 1 && bb.in(myCell) && comp[bb.li(myCell)] >= 0) {
            int main = 0;
            for (int k = 1; k < (int)sz.size(); k++) if (sz[k] > sz[main] || (sz[k] == sz[main] && low[k] < low[main])) main = k;
            cutPiece = comp[bb.li(myCell)] != main;
        }
    }
    if (cutPiece != was) { if (cutPiece) cnt.pieceCuts++; announce(region); }
}
int BbRDRM5Code::extDist() const {             // a cut-off piece: my distance to the main part of my region, through the outside
    int m = INF;
    for (int f = 0; f < MAXF; f++) if (usable(f) && faceRegion[f] != region) m = std::min(m, nbDist[f][region]);
    return m >= DMAX - 1 ? DMAX : m + 1;
}
// advSync: checksum of the distances on link f, as I advertise them (mine) or as I last heard them from the neighbour
int BbRDRM5Code::advHash(int f, bool mine) const {
    unsigned hsh = 2166136261u;
    for (int r = 1; r <= NB_REGIONS; r++) {
        int v = mine ? (r == region && cutPiece && faceRegion[f] != region ? extDist() : dist[r]) : nbDist[f][r];
        v = std::min(v, (int)DMAX); hsh = (hsh ^ (unsigned)(v & 0xFF)) * 16777619u; hsh = (hsh ^ (unsigned)((v >> 8) & 0xFF)) * 16777619u;
    }
    return (int)(hsh & 0xFF);
}
int BbRDRM5Code::advDist(int r, int f) const {
    if (r == region && cutPiece && faceRegion[f] != region) { int e = extDist(); const_cast<BbRDRM5Code *>(this)->extSent = e; return e; }
    return dist[r] >= DMAX ? DMAX : dist[r];
}
// ------------------------------------------------------------------ v5: regions torn in pieces
// My map knows every death around my piece (its members watch those modules and flood the deletions inside it),
// so the parts of the map I cannot reach are other pieces. A piece is named by its lowest cell.
void BbRDRM5Code::pieceInfo() {
    if (pieceVer == mapVersion) return;
    pieceVer = mapVersion; pcomp.clear(); plow.clear(); myLow = -1; nPieces = 1;
    if (!mapReady) return;
    const rmap::BBox &bb = box(); int n = bb.n();
    rmap::Work w; rmap::Bytes v(rmap::bitsBytes(bb), 0); rmap::paintB(v, regionMap, bb, w);
    pcomp.assign(n, -1); std::vector<int> st;
    for (int i = 0; i < n; i++) {
        if (!rmap::get(v, i) || pcomp[i] >= 0) continue;
        int k = (int)plow.size(); plow.push_back(1 << 30); pcomp[i] = k; st.push_back(i);
        while (!st.empty()) {
            int u = st.back(); st.pop_back();
            rmap::C3 c = bb.cellOf(u); plow[k] = std::min(plow[k], rmap::idx(c));
            for (int a = 0; a < 3; a++) for (int sg = -1; sg <= 1; sg += 2) {
                rmap::C3 q = c; q[a] += sg;
                if (!bb.in(q)) continue;
                int j = bb.li(q);
                if (rmap::get(v, j) && pcomp[j] < 0) { pcomp[j] = k; st.push_back(j); }
            }
        }
    }
    peakScratch = std::max<long>(peakScratch, 4L * n);
    nPieces = (int)plow.size();
    if (nPieces > 1 && bb.in(myCell) && pcomp[bb.li(myCell)] >= 0) myLow = plow[pcomp[bb.li(myCell)]];
}
int BbRDRM5Code::pieceOf(int c) {
    pieceInfo();
    if (nPieces < 2 || c < 0) return -1;
    const rmap::BBox &bb = box(); rmap::C3 x = rmap::cell(c);
    if (!bb.in(x)) return -1;
    int k = pcomp[bb.li(x)]; return k < 0 ? -1 : plow[k];
}
bool BbRDRM5Code::inPiece(int key) {
    if (key / 65536 != region) return false;
    pieceInfo();
    if (myLow < 0) return true;                              // my map shows one piece: the region is whole here
    return pieceOf(key % 65536) == myLow;
}
// grad: distance to a piece, like the region gradients (poisoning + hold-down included)
bool BbRDRM5Code::deadEnd(UDataMsg *d) {             // the target's piece is out of reach from here: stop trying
    if (cfg.giveUp != "on" || deadEnds.empty() || region != d->dstRegion) return false;
    int t = d->via >= 0 ? d->realCell : d->dstCell; if (t < 0) return false;
    pieceInfo(); int tp = myLow >= 0 ? pieceOf(t) : -1;
    if (tp < 0 || !deadEnds.count(tp)) return false;
    cnt.deadEndDrops += !reparking; if (d->ack < 0) sendAck(d, 1);
    noteDrop(module->blockId, d->pid); return true;
}
void BbRDRM5Code::needWiden() {                   // a packet waits here for a piece of my region that I cannot hear
    if (cfg.split != "local" || cfg.widen == "off") return;
    wantWiden = true;
    if (!widenTimer) { widenTimer = true; getScheduler()->schedule(new InterruptionEvent<int>(getScheduler()->now() + cfg.widenAfter, module, WIDEN_EVENT)); }
}
void BbRDRM5Code::widenTo(int r, int lim, int fromFace) {
    if (lim > regLim[r]) regLim[r] = lim;
    std::vector<int> ks; for (auto &e : sub) if (e.first / 65536 == r && e.second.lim > 0 && e.second.lim < lim) ks.push_back(e.first);
    for (int k : ks) { sub[k].lim = lim; if (sub[k].seed) subRecalc(k, true); else subRecalc(k); }   // the pieces re-announce; others pass the wider reach on as it comes
    if (fromFace < 0) for (int g = 0; g < MAXF; g++) if (usable(g)) sendCtl(g, new LimMsg(r, lim, lim), K_SPLIT);
}
void BbRDRM5Code::subRecalc(int key, bool force) {
    auto &g = sub[key]; Time now = getScheduler()->now(); int d;
    if (g.seed) d = 0;
    else {
        int m = INF; for (int f = 0; f < MAXF; f++) if (usable(f)) m = std::min(m, g.nb[f]);
        d = (m >= DMAX - 1) ? INF : m + 1;
        if (g.lim && d > g.lim && !(cfg.widen != "off" && key / 65536 == region)) d = INF;   // local: past the announcement's reach (no limit inside the torn region itself)
        if (g.d < INF && d > g.d) { d = INF; g.hold = now + cfg.hold; cnt.poisons++; getScheduler()->schedule(new InterruptionEvent<int>(g.hold, module, SUBHOLD_EVENT)); }
        else if (g.d >= INF && now < g.hold) d = INF;
    }
    if (d == g.d && (!force || d >= INF)) return;
    g.d = d;
    for (int f = 0; f < MAXF; f++) if (usable(f)) sendCtl(f, new SubAdvMsg(key, d >= DMAX ? DMAX : d, g.lim), K_SPLIT);
}
// the key of the piece that holds a cell, as far as I know (grad: a piece announced; seek: a piece found)
int BbRDRM5Code::findKey(int r, int tp, int cell, int gc) {
    if (cfg.split == "grad" || cfg.split == "local") {
        for (auto &e : sub) if (e.first / 65536 == r && e.second.d < INF && !e.second.seed && pieceOf(e.first % 65536) == tp) return e.first;
        return -1;
    }
    if (cfg.split == "walk") {
        for (auto &e : wroutes) if (e.first / 65536 == r && pieceOf(e.first % 65536) == tp) return e.first;
        for (auto &e : routes) if (usable(e.second.face) && e.first / 65536 == r && pieceOf(e.first % 65536) == tp) return e.first;
    }
    if (cfg.split == "seek")
        for (auto &e : routes) if (usable(e.second.face) && e.first / 65536 == r && pieceOf(e.first % 65536) == tp) return e.first;
    int pk = r * 65536 + (cfg.split == "walk" ? cell : tp);   // walk: one question per target cell (my map may merge pieces I cannot tell apart)
    auto fl = seekFailed.find(pk);
    if (fl != seekFailed.end() && getScheduler()->now() - fl->second < (cfg.split == "walk" ? cfg.walkBackoff : cfg.seekBackoff)) return -1;   // asked recently, nobody there
    auto ff = floodFailed.find(pk);
    if (ff != floodFailed.end() && getScheduler()->now() - ff->second < cfg.floodBackoff) return -1;   // even a flood found nobody: gone
    if (cfg.split == "walk") { if (!seekPending.count(pk)) startWalk(pk, cell, gc, -1); return -1; }   // startWalk aims at the piece's nearest cell
    if (!seekPending.count(pk)) startSeek(pk, cell, cfg.seekTtlPct * rmap::manhattan(myCell, rmap::cell(cell)) / 100 + cfg.seekTtlAdd, gc);
    return -1;
}
int BbRDRM5Code::splitFace(UDataMsg *d) {
    if (cfg.split == "grad" || cfg.split == "local") {
        auto it = sub.find(d->sub); if (it == sub.end()) return -1;
        int best = -1, bd = INF; bID bid = 0;
        for (int f = 0; f < MAXF; f++) {
            if (!usable(f) || it->second.nb[f] >= DMAX || it->second.nb[f] >= it->second.d) continue;
            bID id = module->getInterface(f)->getConnectedBlockId();
            if (it->second.nb[f] < bd || (it->second.nb[f] == bd && id < bid)) { best = f; bd = it->second.nb[f]; bid = id; }
        }
        return best;
    }
    auto r = routes.find(d->sub);
    if (r != routes.end() && usable(r->second.face)) return r->second.face;
    if (!seekPending.count(d->sub)) startSeek(d->sub, d->dstCell, 2 * DMAX > 64 ? 64 : 2 * DMAX, d->gc);   // the way broke here: ask around
    return -1;
}
// walk: go around the gap toward a cell of the other piece; the piece that holds it answers with the way
// the cell of a piece (as my map shows it) closest to me: the walk only has to reach the piece
int BbRDRM5Code::nearestCellOf(int low) {
    pieceInfo();
    if (nPieces < 2) return low;
    const rmap::BBox &bb = box(); int k = -1;
    for (size_t i = 0; i < plow.size(); i++) if (plow[i] == low) k = (int)i;
    if (k < 0) return low;
    int best = low, bd = 1 << 30;
    for (int i = 0; i < bb.n(); i++) if (pcomp[i] == k) { rmap::C3 c = bb.cellOf(i); int d = rmap::manhattan(myCell, c); if (d < bd) { bd = d; best = rmap::idx(c); } }
    return best;
}
void BbRDRM5Code::startWalk(int pk, int cell, int gc, int key, int tries) {
    Time now = getScheduler()->now();
    auto fl = seekFailed.find(pk);
    if (fl != seekFailed.end() && now - fl->second < cfg.walkBackoff) return;
    seekInfo[pk] = {cell, gc, tries};
    // aim: odd tries at the nearest cell of the piece (my view), even tries at the packet's own target cell
    if (tries % 2 == 1 && key < 0 && region == pk / 65536) { int tp = pieceOf(cell); if (tp >= 0) cell = nearestCellOf(tp); }
    cnt.walks++; seekSeq = (seekSeq + 1) & 255; logEv("walkstart", module->blockId, seekSeq, cell);
    int base = cfg.walkPct * rmap::manhattan(myCell, rmap::cell(cell)) / 100 + cfg.walkAdd;
    int ttl0 = std::min(3 * DMAX, base * tries);             // a longer detour on the second try
    Time dl = now + (Time)(2 * ttl0 + 10) * 8000;            // out and back, ~8 ms a hop
    seekPending[pk] = {dl, 0};
    getScheduler()->schedule(new InterruptionEvent<int>(dl, module, SEEK_EVENT));
    for (int hand : {1, -1}) {                               // two walkers: the gap on the left, the gap on the right
        WalkMsg w(key, pk / 65536, cell, gc, module->blockId, seekSeq, ttl0); w.hand = hand;
        if (hand < 0) seekSeq = (seekSeq + 1) & 255, w.seq = seekSeq;
        walkStep(&w, -1);
    }
}
void BbRDRM5Code::walkStep(WalkMsg *w, int fromFace) {
    long id = ((long)w->origin << 8) | (w->seq & 255);
    auto seen = walkSeen.find(id);
    if (seen != walkSeen.end() && seen->second <= w->out.size()) {          // been here: cut the loop out of the way
        w->out.resize(seen->second);
        w->back.resize(seen->second);                        // my first visit's way back still holds (the loop is dropped)
    } else walkSeen[id] = w->out.size();
    if (region == w->region && w->origin != module->blockId) {   // am I in the piece that holds the cell?
        pieceInfo();
        bool mine = false;
        if (nPieces < 2) mine = mapReady && rmap::has(regionMap, rmap::cell(w->cell));   // I have not seen the tear yet: I hold it if my map does
        else if (myLow >= 0) {
            int p = pieceOf(w->cell);
            mine = (w->key >= 0 && inPiece(w->key)) || p == myLow;
            if (!mine && p < 0) {                                // the aimed cell may have died: a cell next to my piece will do
                rmap::C3 c = rmap::cell(w->cell);
                for (int a = 0; a < 3 && !mine; a++) for (int sg = -1; sg <= 1; sg += 2) { rmap::C3 n = c; n[a] += sg; if (rmap::inGrid(n) && pieceOf(rmap::idx(n)) == myLow) mine = true; }
            }
        }
        logEv("walkat", module->blockId, w->origin, (mine ? 1 : 0) + 2 * nPieces + 100 * (int)w->back.size());
        if (w->cell == myIdx) mine = true;                  // the aimed cell itself answers
        if (mine && !w->back.empty()) {
            WalkRepMsg *r = new WalkRepMsg(region * 65536 + (myLow >= 0 ? myLow : myIdx), w->origin); r->out = w->out; r->back = w->back; r->want = w->key;   // any cell of my piece names it
            int g = r->back.back(); r->back.pop_back(); sendCtl(g, r, K_SPLIT);
            return;
        }
    }
    if (--w->ttl <= 0) { cnt.walkFail++; logEv("walkttl", module->blockId, w->seq, w->cell); return; }
    cnt.walkHops++; logEv("walk", module->blockId, w->origin, w->follow);
    rmap::C3 tgt = rmap::cell(w->cell);
    int md = rmap::manhattan(myCell, tgt);
    Time tnow = getScheduler()->now();
    auto walkable = [&](int f) { return usable(f) && faceCell[f] >= 0 && !(lastHeard[f] > 0 && tnow - lastHeard[f] > cfg.suspectAfter); };   // a quiet neighbour may be dead: a lost walker costs a retry
    static const int DX[4] = {1, 0, -1, 0}, DZ[4] = {0, -1, 0, 1};
    auto dirOf = [&](int f) { if (faceCell[f] < 0) return -1; rmap::C3 c = rmap::cell(faceCell[f]); int dx = c[0] - myCell[0], dz = c[2] - myCell[2];
                              for (int k = 0; k < 4; k++) if (DX[k] == dx && DZ[k] == dz) return k; return -1; };
    auto greedy = [&]() { int best = -1, bd = md;
        for (int f = 0; f < MAXF; f++) if (walkable(f)) { int dd = rmap::manhattan(rmap::cell(faceCell[f]), tgt); if (dd < bd) { bd = dd; best = f; } }
        return best; };
    int go = -1;
    if (w->follow && md < w->hitDist) { int g = greedy(); if (g >= 0) { w->follow = 0; go = g; } }
    if (go < 0 && !w->follow) {
        go = greedy();
        if (go < 0) {                                        // blocked: keep the wall on the right until closer than here
            w->follow = 1; w->hitCell = myIdx; w->hitDist = md; w->steps = 0; w->hitHeading = -1;
            int toward = std::abs(tgt[0] - myCell[0]) >= std::abs(tgt[2] - myCell[2]) ? (tgt[0] > myCell[0] ? 0 : 2) : (tgt[2] > myCell[2] ? 3 : 1);
            w->heading = (toward + w->hand + 4) % 4;         // as if I had turned away once: the gap is now on my wall side
        }
    }
    if (go < 0) {
        int wallFirst = (4 - w->hand) % 4, away = (4 + w->hand) % 4;
        for (int t : {wallFirst, 0, away, 2}) {                // keep the gap on my wall side: wall side, straight, other side, back
            int dir = (w->heading + t) % 4;
            for (int f = 0; f < MAXF && go < 0; f++) if (walkable(f) && dirOf(f) == dir) go = f;
            if (go >= 0) { w->heading = dir; break; }
        }
        if (w->steps == 0) w->hitHeading = w->heading;
        else if (myIdx == w->hitCell && w->heading == w->hitHeading) { cnt.walkFail++; logEv("walkloop", module->blockId, w->seq, w->cell); return; }   // all the way round: nothing there
        w->steps++;
    }
    if (go < 0) { cnt.walkFail++; return; }
    WalkMsg *n = new WalkMsg(*w); n->out.push_back(go);
    sendCtl(go, n, K_SPLIT);
}
// walk: data toward another piece: inside my piece to the gateway, then the gateway's way, step by step
bool BbRDRM5Code::walkForward(UDataMsg *d, int fromFace) {
    bool broke = false;
    if (!d->sr.empty()) {
        if (d->sri < (int)d->sr.size() && usable(d->sr[d->sri])) { int f = d->sr[d->sri++]; sendData(d, f); return true; }
        d->sr.clear(); d->sri = 0; cnt.walkRepair++; broke = true;   // the way broke here: find a new one from here, never back to the gateway
    }
    auto it = wroutes.find(d->sub);
    if (it != wroutes.end() && !it->second.path.empty()) {
        if (usable(it->second.path[0])) { d->sr = it->second.path; d->sri = 1; d->via = -1; d->wp.clear(); d->wpi = 0; sendData(d, it->second.path[0]); return true; }
        wroutes.erase(it); it = wroutes.end();
    }
    if (!broke && it != wroutes.end() && region == d->sub / 65536 && it->second.gw >= 0 && it->second.gw != myIdx) {
        if (d->via != it->second.gw) {                       // the packet heads for the gateway cell, its real target travels along
            if (d->via < 0) d->realCell = d->dstCell;
            d->via = it->second.gw; d->dstCell = it->second.gw; d->wp.clear(); d->wpi = 0; d->how = -1; d->recov = -1;
        }
        if (inRegionStep(d, fromFace)) return true;
    }
    auto sp = routes.find(d->sub);                                   // a way found by the backup question: hop by hop
    if (sp != routes.end() && usable(sp->second.face)) { d->via = -1; sendData(d, sp->second.face); return true; }
    if (!seekPending.count(d->sub)) startWalk(d->sub, d->via >= 0 ? d->realCell : d->dstCell, d->gc, d->sub);   // no way known here: walk from here
    return false;
}
void BbRDRM5Code::startSeek(int pk, int cell, int ttl, int gc, int tries) {
    Time now = getScheduler()->now();
    cnt.seeks++; seekSeq = (seekSeq + 1) & 255;
    seekPending[pk] = {now + (Time)ttl * 2 * cfg.seekPerHop + 50000, ttl}; seekInfo[pk] = {cell, gc, tries};
    rreqRev[((long)module->blockId << 8) | seekSeq] = {-1, now};
    for (int f = 0; f < MAXF; f++) if (usable(f)) sendCtl(f, new RreqMsg(pk / 65536, cell, module->blockId, seekSeq, ttl, gc), K_SPLIT);
    getScheduler()->schedule(new InterruptionEvent<int>(seekPending[pk].first, module, SEEK_EVENT));
}
// hbAdapt: do I expect fast heartbeats from f? (I depend on it, or I sent it data a moment ago)
bool BbRDRM5Code::fastWatch(int f) const {
    if (cfg.hbAdapt != "on") return true;
    Time now = getScheduler()->now();
    return lastDataOut[f] > 0 && now - lastDataOut[f] < hbBusyW() - cfg.hbPeriod;   // both ends see the same data: no message needed to agree
}
Time BbRDRM5Code::watchFrom(int f) const { return cfg.hbAdapt == "on" ? std::max(lastHeard[f], fastSince[f]) : lastHeard[f]; }
// GPSR on the grid (Karp & Kung 2000): greedy to the target's location; stuck in front of a hole: right-hand rule until closer
bool BbRDRM5Code::gpsrStep(UDataMsg *d, int fromFace) {
    // GPSR (Karp & Kung 2000) on the module grid (a planar graph in 2D): greedy toward the target's location;
    // stuck: perimeter mode on the face crossed by the line Lp -> D, right-hand rule, face change where an edge crosses that line
    // closer to D, back to greedy when closer than Lp, and drop when the first edge of the face comes round again (unreachable)
    int tc = d->dstCell >= 0 ? d->dstCell : (rootCellOf.count(d->dstRegion) ? rootCellOf[d->dstRegion] : -1);
    if (tc < 0) return false;
    rmap::C3 tgt = rmap::cell(tc);
    auto dist2 = [&](double x, double z) { return (x - tgt[0]) * (x - tgt[0]) + (z - tgt[2]) * (z - tgt[2]); };
    double me = dist2(myCell[0], myCell[2]);
    if (d->gpLp >= 0) { rmap::C3 lp = rmap::cell(d->gpLp); if (me < dist2(lp[0], lp[2])) { d->gpLp = -1; d->gpLf = 1e18; } }   // closer than where it got stuck: greedy again
    if (d->gpLp < 0) {
        int best = -1; double bd = me;
        for (int f = 0; f < MAXF; f++) {
            if (!usable(f) || faceCell[f] < 0) continue;
            rmap::C3 c = rmap::cell(faceCell[f]); double dd = dist2(c[0], c[2]);
            if (dd < bd) { best = f; bd = dd; }
        }
        if (best >= 0) { cnt.geoSteps++; sendData(d, best); return true; }
        d->gpLp = myIdx; d->gpLf = me; d->gpE0a = -1; cnt.recoveries++; fromFace = -1;   // enter perimeter mode here
    }
    // neighbours in counter-clockwise order (x right, z up): E, N, W, S
    static const int DX[4] = {1, 0, -1, 0}, DZ[4] = {0, 1, 0, -1};
    int byDir[4] = {-1, -1, -1, -1};
    for (int f = 0; f < MAXF; f++) if (usable(f) && faceCell[f] >= 0) {
        rmap::C3 c = rmap::cell(faceCell[f]); int dx = c[0] - myCell[0], dz = c[2] - myCell[2];
        for (int k = 0; k < 4; k++) if (DX[k] == dx && DZ[k] == dz && c[1] == myCell[1]) byDir[k] = f;
    }
    int start;                                                   // right-hand rule: the first edge counter-clockwise from the edge we came in on
    if (fromFace >= 0 && faceCell[fromFace] >= 0) {
        rmap::C3 c = rmap::cell(faceCell[fromFace]); start = 0;
        for (int k = 0; k < 4; k++) if (DX[k] == c[0] - myCell[0] && DZ[k] == c[2] - myCell[2]) start = k;
    } else {                                                     // entering: start from the direction of the target
        double ang = std::atan2((double)(tgt[2] - myCell[2]), (double)(tgt[0] - myCell[0]));
        start = ((int)std::floor((ang + M_PI / 4) / (M_PI / 2)) % 4 + 4) % 4;
        start = (start + 3) % 4;                                 // so that the first candidate below is that direction
    }
    rmap::C3 lp = rmap::cell(d->gpLp);
    for (int tries = 0; tries < 2; tries++) {
        for (int k = 1; k <= 4; k++) {
            int dir = (start + k) % 4, f = byDir[dir];
            if (f < 0) continue;
            rmap::C3 n = rmap::cell(faceCell[f]);
            // does the edge me -> n cross the segment Lp -> D at a point closer to D than the last face change?
            double x1 = myCell[0], z1 = myCell[2], x2 = n[0], z2 = n[2], x3 = lp[0], z3 = lp[2], x4 = tgt[0], z4 = tgt[2];
            double den = (x1 - x2) * (z3 - z4) - (z1 - z2) * (x3 - x4);
            if (std::fabs(den) > 1e-9) {
                double t = ((x1 - x3) * (z3 - z4) - (z1 - z3) * (x3 - x4)) / den, u = -((x1 - x2) * (z1 - z3) - (z1 - z2) * (x1 - x3)) / den;
                if (t > 1e-9 && t <= 1 && u >= 0 && u <= 1) {
                    double ix = x1 + t * (x2 - x1), iz = z1 + t * (z2 - z1), di = dist2(ix, iz);
                    if (di < d->gpLf - 1e-9) { d->gpLf = di; d->gpE0a = -1; start = dir; break; }   // face change: continue on the next face
                }
            }
            if (d->gpE0a == myIdx && d->gpE0b == faceCell[f]) return false;   // the face's first edge again: the target cannot be reached
            if (d->gpE0a < 0) { d->gpE0a = myIdx; d->gpE0b = faceCell[f]; }
            cnt.perimSteps++; sendData(d, f); return true;
        }
    }
    return false;
}
bool BbRDRM5Code::suspected(int f) const {
    return cfg.suspect == "on" && lastHeard[f] > 0 && fastWatch(f) && getScheduler()->now() - watchFrom(f) > cfg.suspectAfter;
}

// ------------------------------------------------------------------ geocast: anycast, then a flood restricted to the region
void BbRDRM5Code::geocastStep(UDataMsg *d, int fromFace) {
    if (seen.count(d->pid)) return;                          // already have it: a duplicate dies here
    seen.insert(d->pid);
    auto &g = geoRecv[d->pid]; g.first++; g.second = getScheduler()->now(); geoWho[d->pid].insert(module->blockId);
    deliver(d->pid, module->blockId, d->hops, d->hops - d->entryHops, getScheduler()->now());   // first module of the region
    if (cfg.traceDst > 0) setColor(DELIVERED);
    for (int f = 0; f < MAXF; f++)                           // repeat to my region neighbours only
        if (f != fromFace && usable(f) && nbDist[f][region] == 0) sendData(d, f);
}

// ------------------------------------------------------------------ v3: energy, load, balanced next hop
void BbRDRM5Code::spend(double uJ) {
    if (cfg.energy != "on" || !isAlive(module)) return;
    battery -= uJ;
    if (cfg.bye && battery < cfg.reserve * cfg.battery) {   // announce the end while there is energy left to do it
        cnt.byes++; cnt.energyDeaths++;
        if (cnt.firstDeath == 0) cnt.firstDeath = getScheduler()->now();
        battery = 1e18;                                      // (the goodbye itself is paid from the reserve)
        for (int f = 0; f < MAXF; f++) if (usable(f)) sendCtl(f, new ByeMsg(), K_STATE);
        setColor(RED); kill(module);
        return;
    }
    if (battery <= 0) {                                      // empty battery: the module stops, silently, like a failure
        cnt.energyDeaths++;
        if (cnt.firstDeath == 0) cnt.firstDeath = getScheduler()->now();
        setColor(RED); kill(module);
    }
}

// cost of forwarding toward region r through face f. 0 = healthy: nothing to fix.
//   my own link to it (local measure) + its busiest link + its battery + "its way to r is congested"
double BbRDRM5Code::penalty(int f, int r) const {
    // neighbours report only what is above their threshold: level 0 = healthy, 1..7 = how far into the red zone
    double p = hinge(std::min(1.0, linkU[f]), cfg.theta) + (nbLoad[f] / 7.0) * (nbLoad[f] / 7.0);
    if (cfg.energy == "on") p += cfg.deltaE * (nbEnergy[f] / 7.0);     // linear: a module half-way into the red zone is worth a detour
    if ((nbAhead[f] >> r) & 1) p += cfg.lambda;
    return p;
}

int BbRDRM5Code::chooseFace(int r, UDataMsg *d, int fromFace) {
    if (cfg.balance == "none") {
        int b = bestFace(r);
        if (b < 0 || !suspected(b)) return b;
        int alt = -1, ad = INF; bID aid = 0;                 // v5: the usual next hop went quiet: use another closer one if any
        for (int f = 0; f < MAXF; f++) {
            if (f == b || !usable(f) || suspected(f) || nbDist[f][r] >= DMAX || nbDist[f][r] >= dist[r]) continue;
            bID id = module->getInterface(f)->getConnectedBlockId();
            if (nbDist[f][r] < ad || (nbDist[f][r] == ad && id < aid)) { alt = f; ad = nbDist[f][r]; aid = id; }
        }
        if (alt >= 0) { cnt.suspectAvoid++; return alt; }
        return b;
    }
    std::vector<std::pair<bID, int>> C;                      // admissible: strictly closer to r (the paper's C_r(i))
    for (int f = 0; f < MAXF; f++)
        if (usable(f) && nbDist[f][r] < DMAX && nbDist[f][r] < dist[r]) C.push_back({module->getInterface(f)->getConnectedBlockId(), f});
    if (C.empty()) return -1;
    if (cfg.suspect == "on") {                               // v5: drop quiet neighbours while another one is left
        std::vector<std::pair<bID, int>> K; for (auto &c : C) if (!suspected(c.second)) K.push_back(c);
        if (!K.empty() && K.size() < C.size()) { C.swap(K); cnt.suspectAvoid++; }
    }
    std::sort(C.begin(), C.end());
    if (cfg.balance == "rr") return C[rrCount[r]++ % C.size()].second;
    int cur = -1;
    for (auto &c : C) if (c.second == sticky[r]) cur = c.second;
    if (cur < 0) cur = C[0].second;
    double pc = penalty(cur, r);
    if (pc > 0) {                                            // something is wrong with the current next hop
        int best = cur; double pb = pc;
        for (auto &c : C) { double p = penalty(c.second, r); if (p < pb) { pb = p; best = c.second; } }
        if (pc - pb > cfg.hyst) { cur = best; pc = pb; cnt.switches++; }
    }
    if (cur != C[0].second) cnt.awareChoices++;
    sticky[r] = cur;
    if (cfg.balance == "detour" && d && d->det > 0 && pc >= cfg.lambda) {   // every closer neighbour is saturated
        int best = -1; double pb = 1e9;
        for (int f = 0; f < MAXF; f++) {                     // sideways: same distance, not where it came from
            if (!usable(f) || f == fromFace || nbDist[f][r] != dist[r] || dist[r] >= DMAX) continue;
            if (cfg.sig == "hb" && !known[f]) { cnt.blindDetours++; continue; }   // never step sideways blind
            double p = penalty(f, r);
            if (p < pb) { pb = p; best = f; }
        }
        if (best >= 0 && pb + cfg.lambda < pc) { d->det--; cnt.detours++; return best; }
    }
    return cur;
}

// a heartbeat carrying my levels (sig="hb"); an EARLY one is sent outside the period because a level changed
void BbRDRM5Code::sendHb(int f, bool early) {
    HbMsg *h = new HbMsg();
    if (cfg.sig == "hb") {
        h->levels = true; h->load = myLoad; h->energy = myEnergy; h->ahead = myAhead;
        h->keepAhead = (myAhead == toldAhead[f]);            // the mask byte only travels when it changed
        toldLoad[f] = myLoad; toldEnergy[f] = myEnergy; toldAhead[f] = myAhead;
    }
    if (cfg.linkAck == "on" && repDirty[f]) fillRep(f, h);
    if (cfg.advSync == "on" && (hbNo[f]++ & 3) == 0) h->advSum = advHash(f, true);
    if (early) cnt.hbEarly++;
    sendCtl(f, h, K_HB);
}

void BbRDRM5Code::stateTick() {
    double mx = 0;
    for (int f = 0; f < MAXF; f++) {
        double u = txUs[f] / (double)cfg.statePeriod;      // offered load: > 1 means a queue is building up
        linkU[f] = cfg.ewma * u + (1 - cfg.ewma) * linkU[f];
        txUs[f] = 0;
        if (usable(f)) mx = std::max(mx, linkU[f]);
    }
    cnt.maxLinkU = std::max(cnt.maxLinkU, mx);
    // levels are reported relative to the thresholds, so a healthy module never sends anything
    myLoad = mx <= cfg.theta ? 0 : std::min(7, 1 + (int)((std::min(1.0, mx) - cfg.theta) / (1 - cfg.theta) * 7));
    double e = battery / cfg.battery;
    myEnergy = (cfg.energy != "on" || e >= cfg.thetaE) ? 0 : std::min(7, 1 + (int)((cfg.thetaE - e) / cfg.thetaE * 7));
    myAhead = 0;
    if (cfg.balance == "aware" || cfg.balance == "detour") {
        for (int r = 1; r <= NB_REGIONS; r++) {              // all my closer neighbours toward r are saturated: tell upstream
            if (r == region || dist[r] >= DMAX) continue;
            double mn = 1e9; bool any = false;
            for (int f = 0; f < MAXF; f++) if (usable(f) && nbDist[f][r] < dist[r]) { any = true; mn = std::min(mn, penalty(f, r)); }
            if (any && mn >= cfg.lambda) myAhead |= 1 << r;
        }
        if (cfg.sig != "hb") {                               // v3: a STATE message to every neighbour on any change
            if (myLoad != sentLoad || myEnergy != sentEnergy || myAhead != sentAhead) {
                sentLoad = myLoad; sentEnergy = myEnergy; sentAhead = myAhead;
                for (int f = 0; f < MAXF; f++) if (usable(f)) sendCtl(f, new StateMsg(myLoad, myEnergy, myAhead), K_STATE);
            }
        } else {                                             // v4: early heartbeats, only where the news is needed now
            auto zone = [](int l) { return l == 0 ? 0 : l <= 3 ? 1 : 2; };      // healthy / busy / overloaded
            Time now = getScheduler()->now();
            for (int f = 0; f < MAXF; f++) {
                if (!usable(f)) continue;
                bool big = zone(myLoad) != zone(toldLoad[f]) || zone(myEnergy) != zone(toldEnergy[f]);
                bool user = lastDataFrom[f] > 0 && now - lastDataFrom[f] <= cfg.userWin;
                bool small = std::abs(myLoad - toldLoad[f]) >= cfg.deadband || std::abs(myEnergy - toldEnergy[f]) >= cfg.deadband
                             || myAhead != toldAhead[f];
                if (big || (user && small)) { if (big && !user) cnt.hbEarlyAll++; sendHb(f, true); }
            }
        }
    }
}

// a straight greedy walk on the map (no messages): does it reach the target without getting stuck?
// A walk that never gets stuck is monotone, so its length is the Manhattan distance: the shortest possible.
static bool geoWalk(const rmap::Bytes &map, rmap::C3 cur, const rmap::C3 &tgt, rmap::Work &w) {
    while (!(cur == tgt)) {
        int order[3] = {0, 2, 1};
        std::sort(order, order + 3, [&](int x, int y) { return std::abs(tgt[x] - cur[x]) > std::abs(tgt[y] - cur[y]); });
        bool moved = false;
        for (int a : order) {
            if (tgt[a] == cur[a]) continue;
            rmap::C3 n = cur; n[a] += tgt[a] > cur[a] ? 1 : -1;
            if (rmap::hasW(map, n, w)) { cur = n; moved = true; break; }
        }
        if (!moved) return false;
    }
    return true;
}

bool BbRDRM5Code::inRegionStep(UDataMsg *d, int fromFace) {
    const std::string &mode = cfg.inreg;
    if (mode == "flood") {
        if (seen.count(d->pid)) return true;
        seen.insert(d->pid);
        int rm = inRegionMask();
        for (int f = 0; f < MAXF; f++) if ((rm & bit(f)) && f != fromFace) sendData(d, f);
        return true;
    }
    if (mode == "dv") {
        auto it = dvNb.find(d->dstCell); auto me = dvDist.find(d->dstCell);
        if (it == dvNb.end() || me == dvDist.end() || me->second >= INF) { cnt.impossibleFast++; return false; }
        int rm = inRegionMask(), best = -1, bd = INF; bID bid = 0;
        for (int f = 0; f < MAXF; f++) {
            if (!(rm & bit(f)) || it->second[f] >= me->second) continue;
            bID id = module->getInterface(f)->getConnectedBlockId();
            if (it->second[f] < bd || (it->second[f] == bd && id < bid)) { best = f; bd = it->second[f]; bid = id; }
        }
        if (best < 0) return false;
        sendData(d, best); return true;
    }
    rmap::C3 tgt = rmap::cell(d->dstCell);
    // greedy step without any map: the live region neighbour closest to the target (ties: the axis with more to go)
    auto greedyFace = [&]() -> int {
        int rm = inRegionMask(), best = -1, bd = rmap::manhattan(myCell, tgt), bax = -1;
        for (int f = 0; f < MAXF; f++) {
            if (!(rm & bit(f))) continue;
            cnt.faceChecks++;
            rmap::C3 c = rmap::cell(faceCell[f]); int dd = rmap::manhattan(c, tgt);
            int ax = 0; for (int a = 0; a < 3; a++) if (c[a] != myCell[a]) ax = std::abs(tgt[a] - myCell[a]);
            if (dd < bd || (dd == bd && best >= 0 && ax > bax)) { best = f; bd = dd; bax = ax; }
        }
        return best;
    };
    if (mode == "geo") {                                     // textbook geo (GPSR on the grid): no map at all
        int md = rmap::manhattan(myCell, tgt);
        if (d->recov >= 0 && md < d->recov) d->recov = -1;  // closer than where we got stuck: greedy again
        if (d->recov < 0) {
            int f = greedyFace();
            if (f >= 0) { cnt.geoSteps++; sendData(d, f); return true; }
            d->recov = md; cnt.recoveries++;                // stuck in front of a hole: walk along its edge
            fromFace = -1;
        }
        // right-hand rule: turn from the direction I came from, take the first live region neighbour
        static const int DX[4] = {1, 0, -1, 0}, DZ[4] = {0, -1, 0, 1};   // +x, -z, -x, +z (one turning sense)
        auto dirOf = [&](int f) { rmap::C3 c = rmap::cell(faceCell[f]); int dx = c[0] - myCell[0], dz = c[2] - myCell[2];
                                  for (int k = 0; k < 4; k++) if (DX[k] == dx && DZ[k] == dz) return k; return -1; };
        int rm = inRegionMask(), start, k0;
        if (fromFace >= 0 && faceCell[fromFace] >= 0) { start = dirOf(fromFace); k0 = 1; }
        else {                                               // just got stuck: start from the blocked direction toward the target
            int ax = std::abs(tgt[0] - myCell[0]) >= std::abs(tgt[2] - myCell[2]) ? 0 : 2;
            start = ax == 0 ? (tgt[0] > myCell[0] ? 0 : 2) : (tgt[2] > myCell[2] ? 3 : 1); k0 = 0;
        }
        for (int k = k0; k < k0 + 4; k++) {
            int dir = (start + k) % 4;
            for (int f = 0; f < MAXF; f++) {
                if (!(rm & bit(f))) continue;
                cnt.faceChecks++;
                if (dirOf(f) == dir) { cnt.perimSteps++; sendData(d, f); return true; }
            }
        }
        return false;
    }
    if (!mapReady) return false;
    rmap::Work w;
    auto done = [&](bool ok) { cnt.paint += w.paint; cnt.expand += w.expand; cnt.boxTests += w.boxTests;
                               cnt.planWork += w.paint + w.expand; return ok; };
    bool atEntry = d->hops == d->entryHops;
    if (atEntry && !rmap::hasW(regionMap, tgt, w)) { cnt.impossibleFast++; return done(false); }   // not in my map: impossible, fail now
    auto stepToward = [&](const rmap::C3 &goal) -> int {     // one axis-aligned step, x then z then y
        for (int a : {0, 2, 1}) {
            if (goal[a] == myCell[a]) continue;
            rmap::C3 n = myCell; n[a] += goal[a] > myCell[a] ? 1 : -1;
            cnt.faceChecks++;
            return rmap::inGrid(n) ? faceFor(rmap::idx(n)) : -1;   // my own faces tell me: no map reading
        }
        return -1;
    };
    auto bfsHop = [&]() -> int {                             // layered BFS on the map, 2 bits per bbox cell
        std::vector<rmap::C3> mine; std::vector<int> faces;
        int rm = inRegionMask();
        for (int f = 0; f < MAXF; f++) if (rm & bit(f)) { mine.push_back(rmap::cell(faceCell[f])); faces.push_back(f); }
        int k = rmap::nextHop(regionMap, box(), tgt, mine, w);
        cnt.plans++;
        peakScratch = std::max<long>(peakScratch, 3L * rmap::bitsBytes(box()));   // occupancy + 2 layer bits
        return k < 0 ? -1 : faces[k];
    };
    if (mode == "map" || mode == "mapbfs") {
        int f = -1;
        if (mode == "map") {                                  // same clean box as the target: go straight, no search
            int bm = rmap::boxOfW(regionMap, myCell, w), bt = rmap::boxOfW(regionMap, tgt, w);
            if (bm >= 0 && bm == bt) { f = stepToward(tgt); if (f >= 0) cnt.boxSteps++; }
        }
        if (f < 0) f = bfsHop();
        if (f < 0) return done(false);
        sendData(d, f); return done(true);
    }
    // route: waypoints carried by the packet, planned once by the entry cube (cached)
    auto planFrom = [&]() -> bool {
        std::vector<int> wp = rmap::planRoute(regionMap, box(), myCell, tgt, w);
        cnt.plans++;
        peakScratch = std::max<long>(peakScratch, (long)box().n() + rmap::bitsBytes(box()));   // 1 byte per bbox cell + occupancy
        if (wp.empty()) return false;
        d->wp = wp; d->wpi = 0;
        return true;
    };
    if (mode == "adapt" && d->how < 0) {                     // entry cube: go straight only when the map says it is SURE
        if (geoScore >= 1.0) { d->how = 0; cnt.regionGeo++; }        // the region is a block
        else {
            long ops = 0; cnt.geoChecks++;
            if (hullCovered(regionMap, myCell, tgt, ops)) d->how = 0;          // the rectangle from me to the target is all inside
            else {                                                              // else: an L (one turn) that is all inside?
                rmap::C3 cx{tgt[0], myCell[1], myCell[2]}, cz{myCell[0], myCell[1], tgt[2]};
                if (hullCovered(regionMap, myCell, cx, ops) && hullCovered(regionMap, cx, tgt, ops)) d->how = 2;        // x first
                else if (hullCovered(regionMap, myCell, cz, ops) && hullCovered(regionMap, cz, tgt, ops)) d->how = 3;   // z first
                else d->how = 1;                                                // not sure: a plan (below, cached per target)
            }
            w.boxTests += ops;
            if (d->how != 1) cnt.geoChecksOk++;
        }
    }
    if (mode == "adapt" && (d->how == 2 || d->how == 3)) {   // sure along an L: finish one axis, then the other
        static const int ORD[2][3] = {{0, 1, 2}, {2, 0, 1}};   // x first (then y, z) or z first (then x, y): the second box covers the other two axes (3D)
        const int *o = ORD[d->how == 2 ? 0 : 1]; int a = o[2];
        for (int k = 0; k < 3; k++) if (tgt[o[k]] != myCell[o[k]]) { a = o[k]; break; }
        rmap::C3 n = myCell; n[a] += tgt[a] > myCell[a] ? 1 : -1;
        cnt.faceChecks++;
        int f = faceFor(rmap::idx(n));
        if (f >= 0) { cnt.geoSteps++; sendData(d, f); return done(true); }
        d->how = 1; d->wp.clear();
    }
    if (mode == "adapt" && d->how == 0) {                    // sure: plain greedy steps, no map reading
        int f = greedyFace();
        if (f >= 0) { cnt.geoSteps++; sendData(d, f); return done(true); }
        d->how = 1; d->wp.clear();                            // a broken cube nobody reported yet: plan from here
    }
    if (mode == "mapgeo") {   // greedy toward the target's cell, BFS only when stuck
        int md = rmap::manhattan(myCell, tgt);
        if (d->recov >= 0 && md < d->recov) d->recov = -1;      // closer than where we got stuck: greedy again
        if (d->recov < 0) {
            int order[3] = {0, 2, 1};
            std::sort(order, order + 3, [&](int x, int y) { return std::abs(tgt[x] - myCell[x]) > std::abs(tgt[y] - myCell[y]); });
            for (int a : order) {
                if (tgt[a] == myCell[a]) continue;
                rmap::C3 n = myCell; n[a] += tgt[a] > myCell[a] ? 1 : -1;
                if (!rmap::hasW(regionMap, n, w)) continue;
                int f = faceFor(rmap::idx(n));
                if (f < 0) continue;
                cnt.geoSteps++; sendData(d, f); return done(true);
            }
            d->recov = md; cnt.recoveries++;                  // stuck: a wall or a hole is in the way
        }
        int f = bfsHop();
        if (f < 0) return done(false);
        sendData(d, f); return done(true);
    }
    if (d->wp.empty()) {
        bool hit = false;
        for (auto it = cache.begin(); it != cache.end(); ++it)
            if (it->first == d->dstCell && it->second.first == mapVersion) {
                d->wp = it->second.second; d->wpi = 0; cnt.cacheHits++; hit = true;
                cache.splice(cache.begin(), cache, it);
                break;
            }
        if (!hit) {
            if (!planFrom()) return done(false);
            cache.push_front({d->dstCell, {mapVersion, d->wp}});
            while ((int)cache.size() > cfg.cacheSize) cache.pop_back();
        }
    }
    for (int attempt = 0; attempt < 2; attempt++) {
        while (d->wpi < (int)d->wp.size() && d->wp[d->wpi] == myIdx) d->wpi++;
        int f = d->wpi < (int)d->wp.size() ? stepToward(rmap::cell(d->wp[d->wpi])) : -1;
        if (f >= 0) { cnt.routeSteps++; sendData(d, f); return done(true); }
        cnt.reroutes++;                                       // segment blocked (a known hole): plan again from here
        d->wp.clear();
        if (!planFrom()) return done(false);
    }
    return done(false);
}

// ------------------------------------------------------------------ report
long BbRDRM5Code::memBytes(long &grad, long &hb, long &inreg, long &scratch) const {
    grad = NB_REGIONS;
    for (int f = 0; f < MAXF; f++) if (usable(f)) for (int r = 1; r <= NB_REGIONS; r++) if (nbDist[f][r] < DMAX) grad++;
    hb = cfg.detect == "hb" ? 2 + 2 * popc(subMask) : 0;
    inreg = 0;
    if (cfg.traffic == "unicast") {
        if (cfg.inreg == "flood" || cfg.traffic == "geocast") inreg = 4 * (long)seen.size();
        if (cfg.inreg == "dv") {
            inreg = 3 * (long)dvDist.size();
            for (auto &e : dvNb) for (int f = 0; f < MAXF; f++) if (e.second[f] < INF) inreg++;
        }
        if (usesMap()) inreg = regionMap.size();
        if (cfg.inreg == "route" || cfg.inreg == "adapt") for (auto &e : cache) inreg += 2 + 2 * (long)e.second.second.size();
    }
    if (cfg.traffic == "geocast") inreg = 4 * (long)seen.size();      // ids of the geocasts already repeated
    grad += 10 * (long)sub.size() + 4 * (long)routes.size();
    for (auto &w : wroutes) grad += 3 + faceBits(w.second.path.size());   // split: piece key 3 B + distance + 6 neighbour distances / key 2 B + face + hops
    scratch = peakScratch;
    return grad + hb + inreg;
}

std::string BbRDRM5Code::onInterfaceDraw() {
    std::stringstream s;
    s << "RDRM5 " << cfg.traffic << "/" << cfg.inreg << " detect=" << cfg.detect
      << "  adv=" << cnt.msgs[K_ADV] << " hb=" << cnt.msgs[K_HB] << " map=" << cnt.msgs[K_MAP] + cnt.msgs[K_MAPDEL]
      << " dv=" << cnt.msgs[K_DV] << " data=" << cnt.msgs[K_DATA] << " packets=" << packets.size();
    return s.str();
}

void BbRDRM5Code::onEndOfSimulation() {
    if (reported) return;
    reported = true;
    auto &blocks = BaseSimulator::getWorld()->getMap();
    long N = 0, E2 = 0;
    for (auto &b : blocks) if (isAlive(b.second)) {
        N++;
        for (auto *itf : b.second->getP2PNetworkInterfaces())
            if (itf->isConnected() && isAlive(BaseSimulator::getWorld()->getBlockById(itf->getConnectedBlockId()))) E2++;
    }
    // oracle: true shortest path on the final alive graph
    std::map<bID, std::map<bID, int>> toDst; std::map<int, std::map<bID, int>> toRegion;
    long okc = 0, deliverable = 0; double sH = 0, sO = 0, sIn = 0, sLat = 0; long inN = 0; Time maxLat = 0;
    std::ofstream f("results_rdrm5_" + cfg.name + ".csv");
    f << "pid,src,srcRegion,dstRegion,dst,hops,oracle,inHops,tx,latency_us,sent_us,retx\n";
    for (size_t i = 0; i < packets.size(); i++) {
        PacketRecord &p = packets[i];
        int o = -1;
        if (p.dst) {
            BuildingBlock *db = BaseSimulator::getWorld()->getBlockById(p.dst);
            if (isAlive(db)) { if (!toDst.count(p.dst)) toDst[p.dst] = bfsFrom({p.dst}); auto &m = toDst[p.dst]; if (m.count(p.src)) o = m[p.src]; }
        } else {
            if (!toRegion.count(p.dstRegion)) { std::vector<bID> s; for (bID b : members[p.dstRegion]) if (isAlive(BaseSimulator::getWorld()->getBlockById(b))) s.push_back(b); toRegion[p.dstRegion] = bfsFrom(s); }
            auto &m = toRegion[p.dstRegion]; if (m.count(p.src)) o = m[p.src];
        }
        if (o >= 0) deliverable++;
        f << i << "," << p.src << "," << p.srcRegion << "," << p.dstRegion << "," << p.dst << "," << p.hops << "," << o << ","
          << p.inHops << "," << p.tx << "," << (p.ok ? (long long)(p.delivered - p.sent) : -1) << "," << p.sent << "," << p.retx << "\n";
        if (!p.ok && o >= 0 && getenv("RR_LOSTDBG")) {   // debug: where is a lost deliverable packet?
            int inP = 0, tr = -1, gn = -1; BuildingBlock *sb = BaseSimulator::getWorld()->getBlockById(p.src);
            if (sb) for (auto &q : static_cast<BbRDRM5Code *>(sb->blockCode)->pend) if (q.second.msg.pid == (int)i) { inP = 1; tr = q.second.tries; gn = q.second.gone; }
            std::cout << "LOST pid=" << i << " src=" << p.src << " dst=" << p.dst << " sr=" << p.srcRegion << " dr=" << p.dstRegion << " sent=" << p.sent / 1000 << " tx=" << p.tx
                      << " retx=" << p.retx << " inPend=" << inP << " tries=" << tr << " gone=" << gn << " oracle=" << o << "\n";
        }
        if (!p.ok || o < 0) continue;
        okc++; sH += p.hops; sO += o; Time lat = p.delivered - p.sent; sLat += lat; maxLat = std::max(maxLat, lat);
        if (p.inHops >= 0) { sIn += p.inHops; inN++; }
    }
    f.close();
    if (evlog) evlog->flush();
    // memory and load per module
    double sMem = 0, sG = 0, sHb = 0, sIr = 0; long maxMem = 0, maxIr = 0, maxScr = 0, maxTx = 0; double sTx = 0, sBoxes = 0; long nBoxes = 0;
    std::ofstream fm("modules_rdrm5_" + cfg.name + ".csv");
    fm << "id,x,y,z,region,tx,memBytes,gradB,hbB,inregB,scratchB,alive\n";
    for (auto &b : blocks) {
        BbRDRM5Code *c = static_cast<BbRDRM5Code *>(b.second->blockCode);
        long g, h, ir, sc; long mem = c->memBytes(g, h, ir, sc);
        long tx = cnt.txPerModule.count(b.first) ? cnt.txPerModule.at(b.first) : 0;
        const Cell3DPosition &p = b.second->position;
        fm << b.first << "," << p[0] << "," << p[1] << "," << p[2] << "," << regionOf[b.first] << "," << tx << ","
           << mem << "," << g << "," << h << "," << ir << "," << sc << "," << isAlive(b.second) << "\n";
        if (!isAlive(b.second)) continue;
        sMem += mem; sG += g; sHb += h; sIr += ir; maxMem = std::max(maxMem, mem); maxIr = std::max(maxIr, ir); maxScr = std::max(maxScr, sc);
        sTx += tx; maxTx = std::max(maxTx, tx);
        if (c->mapReady) { sBoxes += c->regionMap.size() / 4.0; nBoxes++; }
    }
    fm.close();
    double P = packets.size();
    long ctl = 0, ctlB = 0; for (int k = 0; k < K_N; k++) if (k != K_DATA && k != K_ACK) { ctl += cnt.msgs[k]; ctlB += cnt.bytes[k]; }
    std::cout << std::fixed; std::cout.precision(3);
    // v3 load-spread and latency statistics
    std::vector<double> lats; for (auto &p : packets) if (p.ok) lats.push_back((p.delivered - p.sent) / 1000.0);
    std::sort(lats.begin(), lats.end());
    double p95 = lats.empty() ? 0 : lats[(size_t)(0.95 * (lats.size() - 1))], p50 = lats.empty() ? 0 : lats[lats.size() / 2];
    double s1 = 0, s2 = 0; long nr = 0, maxData = 0;
    for (auto &e : cnt.dataTx) if (e.second > 0) { s1 += e.second; s2 += (double)e.second * e.second; nr++; maxData = std::max(maxData, e.second); }
    long okAll = 0; Time lastOk = 0; for (auto &p : packets) if (p.ok) { okAll++; lastOk = std::max(lastOk, p.delivered); }
    long aliveEnd = 0; for (auto &b : blocks) if (isAlive(b.second)) aliveEnd++;
    // geocast statistics: coverage of the live modules of the target region
    double covSum = 0, covMin = 1, doneSum = 0, covAfter = 0; long gN = 0, nAfter = 0;
    if (cfg.traffic == "geocast") {
        std::map<int, long> liveIn; for (auto &b : blocks) if (isAlive(b.second)) liveIn[regionOf[b.first]]++;
        for (size_t i = 0; i < packets.size(); i++) {
            auto it = geoRecv.find((int)i); long got = it == geoRecv.end() ? 0 : it->second.first;
            double c = liveIn[packets[i].dstRegion] ? std::min(1.0, (double)got / liveIn[packets[i].dstRegion]) : 0;
            covSum += c; covMin = std::min(covMin, c); gN++;
            if (cfg.failAt > 0 && packets[i].sent >= cfg.failAt + 2000000) { covAfter += c; nAfter++; }   // after detection
            if (it != geoRecv.end()) doneSum += (it->second.second - packets[i].sent) / 1000.0;
        }
    }
    double rcSum = 0, rcMin = 1; long rcN = 0;                 // coverage of the region modules the sender can still reach, after detection
    if (cfg.traffic == "geocast" && cfg.failAt > 0) {
        for (size_t i = 0; i < packets.size(); i++) {
            if (packets[i].sent < cfg.failAt + 2000000) continue;
            BuildingBlock *sb = BaseSimulator::getWorld()->getBlockById(packets[i].src);
            if (!isAlive(sb)) continue;
            auto reach = bfsFrom({packets[i].src});
            long want = 0, have = 0; auto &who = geoWho[(int)i];
            for (bID b : members[packets[i].dstRegion]) if (reach.count(b)) { want++; if (who.count(b)) have++; }
            if (!want) continue;
            double c = (double)have / want; rcSum += c; rcMin = std::min(rcMin, c); rcN++;
            if (c < 0.9 && getenv("RR_GEODBG")) { std::cout << "GEOWORST pid=" << i << " src=" << packets[i].src << " sent=" << packets[i].sent << " region=" << packets[i].dstRegion << " cov=" << c << " missing=";
                int k = 0; for (bID b : members[packets[i].dstRegion]) if (reach.count(b) && !who.count(b) && k++ < 8) std::cout << b << ","; std::cout << "\n"; }
        }
    }
    std::cout << "REACHCOV n=" << rcN << " reachCov=" << (rcN ? rcSum / rcN : 1) << " reachCovMin=" << (rcN ? rcMin : 1) << "\n";
    std::cout << "GEOCAST geocasts=" << gN << " avgCoverage=" << (gN ? covSum / gN : 0) << " minCoverage=" << (gN ? covMin : 0)
              << " avgCompletion_ms=" << (gN ? doneSum / gN : 0) << " msgsPerGeocast=" << (gN ? cnt.msgs[K_DATA] / (double)gN : 0)
              << " fullFloodMsgs=" << (E2 - N + 1)
              << " geocastsAfterDetection=" << nAfter << " coverageAfterDetection=" << (nAfter ? covAfter / nAfter : 0) << "\n";
    {
        double sp = 0; long np = 0, mp = 0;
        for (auto &b : blocks) { BbRDRM5Code *c = static_cast<BbRDRM5Code *>(b.second->blockCode); if (c->peakBufB > 0) { sp += c->peakBufB; np++; mp = std::max(mp, c->peakBufB); } }
        std::cout << "RESULT5 salvage=" << cfg.salvage << " suspect=" << cfg.suspect << " salvageWin_ms=" << cfg.salvageWin / 1000.0
                  << " salvaged=" << cnt.salvaged << " resentMsgs=" << cnt.resentMsgs << " parked=" << cnt.parked << " parkExpired=" << cnt.parkExpired
                  << " parkFull=" << cnt.parkFull << " logFull=" << cnt.logFull << " advReq=" << cnt.advReq << " advResync=" << cnt.advResync << " dupDiscards=" << cnt.dupDiscards << " dupDeliveries=" << cnt.dupDeliveries
                  << " suspectAvoid=" << cnt.suspectAvoid << " avgPeakBufB=" << (np ? sp / np : 0) << " maxPeakBufB=" << mp << " dropInreg=" << cnt.dropInreg << " dropHops=" << cnt.dropHops << " inregParked=" << cnt.inregParked << " escapes=" << cnt.escapes << " escapeHops=" << cnt.escapeHops << " escapeDead=" << cnt.escapeDead << " pieceCuts=" << cnt.pieceCuts << " pieceExits=" << cnt.pieceExits << " split=" << cfg.split << " splitRedirects=" << cnt.splitRedirects << " splitFanout=" << cnt.splitFanout << " splitParked=" << cnt.splitParked << " subKeys=" << cnt.subKeys << " seeks=" << cnt.seeks << " linkLoss=" << cfg.linkLoss << " linkLostCtl=" << cnt.linkLostCtl << " linkLostData=" << cnt.linkLostData << " quietParked=" << cnt.quietParked << " splitRenamed=" << cnt.splitRenamed << " walks=" << cnt.walks << " walkFallbacks=" << cnt.walkFallbacks << " walkFound=" << cnt.walkFound << " walkFail=" << cnt.walkFail << " walkHops=" << cnt.walkHops << " gwAnnounce=" << cnt.gwAnnounce << " walkRepair=" << cnt.walkRepair << " seekReplies=" << cnt.seekReplies << " seekGiveUp=" << cnt.seekGiveUp << " escapeGiveUp=" << cnt.escapeGiveUp << " escapeOk=" << cnt.escapeOk << " escapeOkLeft=" << cnt.escapeOkHops << "\n";
    }
    {
        long pendEnd = 0; for (auto &b : blocks) if (isAlive(b.second)) pendEnd += static_cast<BbRDRM5Code *>(b.second->blockCode)->pend.size();
        long acksOk = 0; for (auto &a : acks) if (a.ok) acksOk++;
        long rD = 0, rU = 0, txD = 0, txU = 0, uN[3] = {0}, uR[3] = {0}, uT[3] = {0};          // resends and data messages of the packets that could / could not arrive (harness view)
        for (size_t i = 0; i < packets.size(); i++) { PacketRecord &p = packets[i]; int o = -1;
            if (p.dst) { auto it = toDst.find(p.dst); if (it != toDst.end() && it->second.count(p.src)) o = 0; }
            else { auto it = toRegion.find(p.dstRegion); if (it != toRegion.end() && it->second.count(p.src)) o = 0; }
            if (o >= 0) { rD += p.retx; txD += p.tx; } else { rU += p.retx; txU += p.tx;
                bool sa = isAlive(BaseSimulator::getWorld()->getBlockById(p.src)), da = !p.dst || isAlive(BaseSimulator::getWorld()->getBlockById(p.dst));
                int c = !sa ? 0 : !da ? 1 : 2; uN[c]++; uR[c] += p.retx; uT[c] += p.tx; } }
        { long rl = 0, rlOk = 0, be = 0, beOk = 0;
          for (size_t i = 0; i < packets.size(); i++) { PacketRecord &p = packets[i]; if (cfg.traffic == "geocast") break; bool dlv = false;
            if (p.dst) { auto it = toDst.find(p.dst); dlv = it != toDst.end() && it->second.count(p.src); }
            else { auto it = toRegion.find(p.dstRegion); dlv = it != toRegion.end() && it->second.count(p.src); }
            if (!dlv) continue; if (p.rel) { rl++; rlOk += p.ok; } else { be++; beOk += p.ok; } }
          std::cout << "E2EREL relDeliverable=" << rl << " relDelivered=" << rlOk << " beDeliverable=" << be << " beDelivered=" << beOk << "\n"; }
        std::cout << "E2EUND srcDead=" << uN[0] << "/" << uR[0] << "/" << uT[0] << " dstDead=" << uN[1] << "/" << uR[1] << "/" << uT[1] << " cut=" << uN[2] << "/" << uR[2] << "/" << uT[2] << "\n";
        std::cout << "E2ESPLIT resentDeliverable=" << rD << " resentUndeliverable=" << rU << " dataTxDeliverable=" << txD << " dataTxUndeliverable=" << txU << "\n";
        std::cout << "RESULT6 e2e=" << cfg.e2e << " copyLog=" << cfg.copyLog << " rto_ms=" << cfg.rto / 1000.0 << " tail_ms=" << cfg.tail / 1000.0
                  << " e2eResent=" << cnt.e2eResent << " e2eGiveUp=" << cnt.e2eGiveUp << " e2eAcked=" << cnt.e2eAcked << " e2eGone=" << cnt.e2eGone << " e2eNoway=" << cnt.e2eNoway << " hbSlowSkipped=" << cnt.hbSlowSkipped << " deadEndDrops=" << cnt.deadEndDrops << " unreachSent=" << cnt.unreachSent << " unreachStop=" << cnt.unreachStop << " linkReleased=" << cnt.linkReleased << " linkRetx=" << cnt.linkRetx << " linkDup=" << cnt.linkDup << " linkReports=" << cnt.linkReports << " mapDelBatches=" << cnt.mapDelBatches << " widens=" << cnt.widens << " widenChecksDeadPiece=" << cnt.widenDead << " widenMax=" << cnt.widenMax
                  << " acksSent=" << cnt.acksSent << " nacksSent=" << cnt.nacksSent << " acksArrived=" << acksOk << " e2eLate=" << cnt.e2eLate
                  << " pendEnd=" << pendEnd << " e2ePeak=" << cnt.e2ePeak << " ackMsgs=" << cnt.msgs[K_ACK] << " ackBytes=" << cnt.bytes[K_ACK] << "\n";
    }
    std::cout << "RESULT4 sig=" << cfg.sig << " hbPiggy=" << cfg.hbPiggy << " deadband=" << cfg.deadband << " userWin_ms=" << cfg.userWin / 1000.0
              << " hbMsgs=" << cnt.msgs[K_HB] << " hbBytes=" << cnt.bytes[K_HB] << " hbEarly=" << cnt.hbEarly << " hbEarlyAll=" << cnt.hbEarlyAll
              << " hbSkipped=" << cnt.hbSkipped << " stateMsgs=" << cnt.msgs[K_STATE] << " signalMsgs=" << cnt.msgs[K_HB] + cnt.msgs[K_STATE]
              << " signalBytes=" << cnt.bytes[K_HB] + cnt.bytes[K_STATE] << " blindDetoursAvoided=" << cnt.blindDetours << "\n";
    std::cout << "RESULT3 balance=" << cfg.balance << " energy=" << cfg.energy << " theta=" << cfg.theta << " lambda=" << cfg.lambda
              << " detours=" << cnt.detours << " switches=" << cnt.switches << " awareChoices=" << cnt.awareChoices
              << " stateMsgs=" << cnt.msgs[K_STATE] << " stateBytes=" << cnt.bytes[K_STATE]
              << " p50Latency_ms=" << p50 << " p95Latency_ms=" << p95 << " maxLinkU=" << cnt.maxLinkU
              << " relays=" << nr << " maxDataTx=" << maxData << " jainTx=" << (nr ? s1 * s1 / (nr * s2) : 0)
              << " byes=" << cnt.byes << " energyDeaths=" << cnt.energyDeaths << " firstDeath_ms=" << cnt.firstDeath / 1000.0 << " aliveEnd=" << aliveEnd << " deliveredAll=" << okAll << " lastDelivery_ms=" << lastOk / 1000.0 << "\n";
    std::cout << "RESULT2 algo=rdrm5 config=" << cfg.name << " traffic=" << cfg.traffic << " inreg=" << cfg.inreg
              << " detect=" << cfg.detect << " hbPeriod_ms=" << cfg.hbPeriod / 1000.0 << " targets=" << cfg.targets
              << " N=" << N << " E=" << E2 / 2 << " packets=" << packets.size() << " delivered=" << okc << " deliverable=" << deliverable
              << " killed=" << killed.size() << " drops=" << cnt.drops << " lostAtDead=" << cnt.lostAtDead;
    for (int k = 0; k < K_N; k++) std::cout << " " << KIND[k] << "Msgs=" << cnt.msgs[k];
    for (int k = 0; k < K_N; k++) std::cout << " " << KIND[k] << "Bytes=" << cnt.bytes[k];
    for (int k = 0; k < K_N; k++) std::cout << " " << KIND[k] << "AfterFail=" << cnt.afterFail[k];
    double dur = cfg.stopAt / 1e6;
    std::cout << " ctlMsgs=" << ctl << " ctlBytes=" << ctlB
              << " hbPerSec=" << cnt.msgs[K_HB] / dur
              << " dataMsgsPerPacket=" << (P ? cnt.msgs[K_DATA] / P : 0) << " dataBytesPerPacket=" << (P ? cnt.bytes[K_DATA] / P : 0)
              << " avgHops=" << (okc ? sH / okc : 0) << " avgOracle=" << (okc ? sO / okc : 0)
              << " stretch=" << (sO > 0 ? sH / sO : 0) << " avgInHops=" << (inN ? sIn / inN : 0)
              << " avgLatency_ms=" << (okc ? sLat / okc / 1000.0 : 0) << " maxLatency_ms=" << maxLat / 1000.0
              << " plans=" << cnt.plans << " planWork=" << cnt.planWork << " boxSteps=" << cnt.boxSteps
              << " routeSteps=" << cnt.routeSteps << " cacheHits=" << cnt.cacheHits << " reroutes=" << cnt.reroutes
              << " paint=" << cnt.paint << " expand=" << cnt.expand << " boxTests=" << cnt.boxTests
              << " workPerPacket=" << (P ? (cnt.paint + cnt.expand + cnt.boxTests + cnt.faceChecks) / P : 0) << " faceChecks=" << cnt.faceChecks << " perimSteps=" << cnt.perimSteps
              << " geoChecks=" << cnt.geoChecks << " geoChecksOk=" << cnt.geoChecksOk << " regionGeo=" << cnt.regionGeo << " analysisWork=" << cnt.analysisWork
              << " geoSteps=" << cnt.geoSteps << " recoveries=" << cnt.recoveries << " subPiggy=" << cnt.subPiggy
              << " poisons=" << cnt.poisons << " holdWakes=" << cnt.holdWakes << " impossibleFast=" << cnt.impossibleFast
              << " marked=" << cnt.marked << " waves=" << cnt.waves << " hbScope=" << cfg.hbScope << " cti=" << cfg.cti << " leader=" << cfg.leader
              << " merge=" << (cfg.mergeEcho ? "all" : (cfg.mergeSub ? "sub" : "none"))
              << " detections=" << cnt.detections << " avgDetect_ms=" << (cnt.detections ? cnt.detectDelaySum / cnt.detections / 1000.0 : 0)
              << " maxDetect_ms=" << cnt.detectMax / 1000.0 << " falsePos=" << cnt.falsePos << " resurrect=" << cnt.resurrect
              << " avgMemB=" << sMem / N << " maxMemB=" << maxMem << " avgGradB=" << sG / N << " avgHbB=" << sHb / N
              << " avgInregB=" << sIr / N << " maxInregB=" << maxIr << " peakScratchB=" << maxScr
              << " avgBoxes=" << (nBoxes ? sBoxes / nBoxes : 0)
              << " avgTx=" << sTx / N << " maxTx=" << maxTx
              << " advEnd_ms=" << cnt.lastAdv / 1000.0 << " inregEnd_ms=" << cnt.lastInreg / 1000.0
              << " start_ms=" << cfg.start / 1000.0 << " failAt_ms=" << cfg.failAt / 1000.0 << " stopAt_ms=" << cfg.stopAt / 1000.0 << "\n";
    if (cfg.inreg == "adapt")
        for (auto &b : blocks) { BbRDRM5Code *c = static_cast<BbRDRM5Code *>(b.second->blockCode);
            if (c->isLeader && c->geoScore >= 0) std::cout << "REGIONGEO R" << c->region << " boxes=" << c->regionMap.size() / 4 << " geoScore=" << c->geoScore << "\n"; }
    if (cnt.lastInreg > cfg.start && (cfg.failAt == 0 || cnt.lastInreg < cfg.failAt))
        std::cout << "WARNING in-region setup still running when traffic started\n";
}
