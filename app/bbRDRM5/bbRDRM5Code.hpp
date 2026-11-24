/**
 * @file bbRDRM5Code.hpp
 * RDRM v4 = v3 (gradients, heartbeats, poisoning, region maps, unicast, geocast, load/energy balancing)
 * + the load and energy levels travel INSIDE the heartbeats (sig="hb"):
 *   - every heartbeat (1 byte) carries my load level (3 bits) and energy level (3 bits); a second byte
 *     carries the congested-ahead mask, only when it is not empty. No separate STATE message.
 *   - normal speed: the periodic heartbeat to every neighbour (every hbPeriod) refreshes my levels.
 *   - fast: when my levels change, an EARLY heartbeat goes at once
 *       . to my USERS (neighbours that sent me data in the last userWin) when a level moved by >= deadband
 *         or the congested-ahead mask changed;
 *       . to EVERY neighbour when a level changes zone (healthy 0 / busy 1-3 / overloaded 4-7).
 *   - a periodic heartbeat is skipped on a face that carried another message this period, if that
 *     neighbour already knows my current levels (any message is a sign of life).
 *   - a sideways detour only goes to a neighbour whose levels I have heard at least once.
 * sig="state" restores the v3 behaviour (separate 2-byte STATE messages to every neighbour) for comparison.
 *
 * Inherited description (v2/v3):
 *
 * GRADIENTS (as RDRM v1). For every region r each module keeps D_r = hops to the
 * nearest module of r, recomputed from its neighbours' announced values.
 * The neighbour it forwards to for region r (strictly lower, lowest id) is its
 * PARENT for r; the module is that parent's CHILD.
 *
 * HEARTBEATS (detect="hb"). A child tells each of its parents once "I depend on
 * you" (SUB, 1 byte) and "no longer" when it switches. Every hbPeriod a parent
 * sends ONE heartbeat (1 byte) on each face that has a child, whatever the number
 * of regions served. Any message from a face also counts as a sign of life.
 * A child that hears nothing from a parent for hbMiss periods declares it dead:
 * forgets its values, recomputes (another lower neighbour, or a longer path),
 * and if the dead cube belonged to its own region, removes it from the region map.
 * Failures are SILENT (failMode="silent"): the cube stops, links stay plugged, the
 * simulator tells nobody.
 *
 * UNICAST. A packet names its target cube by its cell. It travels like an anycast
 * to the target's region, then navigates inside the region (inreg=):
 *   flood  : every cube of the region repeats it once            (no table)
 *   dv     : every cube keeps the distance to every cube of its region (distance vector)
 *   map    : every cube keeps the region map (packed boxes); each hop goes straight
 *            if it shares a box with the target, otherwise a layered BFS on the map
 *   mapbfs : the same map, a BFS on every hop (reference for the computation cost)
 *   mapgeo : the same map, greedy step toward the target's cell while the next cell is
 *            in the map; BFS only when stuck, until closer than where it got stuck (GPSR-like)
 *   adapt  : the entry cube reads the map first: if a straight greedy walk on the map reaches
 *            the target (no computation beyond box tests), the packet goes geo; otherwise
 *            the entry cube plans the route once (waypoints). The decision is cached per target.
 *   route  : the entry cube plans ONCE, puts the waypoints (turn cells) in the packet
 *            and caches them; the other cubes just follow the straight segments
 * The map is built once per region by an echo wave from the region's leader
 * (convergecast of packed boxes, then broadcast of the merged map). Planning scratch
 * covers the region's bounding box only.
 *
 * MERGING WITH THE GRADIENT SETUP (merge=):
 *   sub  : subscriptions are a flag in the ADV the child sends anyway (standalone SUB only
 *          when no ADV goes to that parent)                              [default]
 *   all  : + the map wave IS the region's own ADV(r,0): the leader starts it, a cube
 *          announces its region when the wave reaches it, the parent is flagged in the ADV
 *   none : separate SUB and map-wave messages
 *
 * author: Nawar ELZEIN
 **/

#ifndef bbRDRM5Code_H_
#define bbRDRM5Code_H_

#include "rdrm5Common.hpp"
#include "rdrm2Map.hpp"
#include <list>
#include <deque>
#include <unordered_set>

static const int ADV_ID = 5201, SUB_ID = 5202, HB_ID = 5203, MTREE_ID = 5204, MECHO_ID = 5205,
                 MDOWN_ID = 5206, MDEL_ID = 5207, DV_ID = 5208, STATE_ID = 5209, BYE_ID = 5210, UDATA_ID = 5002,
                 SUBADV_ID = 5211, RREQ_ID = 5212, RREP_ID = 5213, WALK_ID = 5214, WALKREP_ID = 5215, GW_ID = 5216, LIM_ID = 5217, MADD_ID = 5218;
// split="grad": a piece of a torn region announces its own gradient. 4 bytes: [region 3 bits | piece cell 13 bits] [piece cell] [distance]
struct SubAdvMsg : Message { int key, dist, lim = 0;           // local: + 1 byte, how far the announcement may travel
    SubAdvMsg(int k, int d, int l = 0) : key(k), dist(d), lim(l) { type = SUBADV_ID; }
    unsigned int size() const override { return lim ? 5 : 4; } Message *clone() const override { return new SubAdvMsg(*this); } };
// "this cell is alive after all" (a neighbour declared dead was only late, e.g. on a lossy link): 2 bytes
struct MapAddMsg : Message { int cell, ver; MapAddMsg(int c, int v) : cell(c), ver(v) { type = MADD_ID; }
    unsigned int size() const override { return 3; } Message *clone() const override { return new MapAddMsg(*this); } };
// local: "pieces of region r, announce yourselves this far" (a module of r cannot hear one of its pieces). 3 bytes
struct LimMsg : Message { int region, lim, ttl;
    LimMsg(int r, int l, int t) : region(r), lim(l), ttl(t) { type = LIM_ID; }
    unsigned int size() const override { return 3; } Message *clone() const override { return new LimMsg(*this); } };
// split="seek": "who holds this cell of region r?" 7 bytes: region+cell 2, origin 2, seq 1, ttl 1, flags 1
struct RreqMsg : Message { int region, cell, ttl, seq, gc; bID origin;
    RreqMsg(int r, int c, bID o, int s, int t, int g) : region(r), cell(c), ttl(t), seq(s), gc(g), origin(o) { type = RREQ_ID; }
    unsigned int size() const override { return 7; } Message *clone() const override { return new RreqMsg(*this); } };
// the answer, back along the way the question came: 6 bytes: piece key 2, origin 2, seq 1, hops 1
struct RrepMsg : Message { int key, hops, seq; bID origin;
    RrepMsg(int k, bID o, int s, int h) : key(k), hops(h), seq(s), origin(o) { type = RREP_ID; }
    unsigned int size() const override { return 6; } Message *clone() const override { return new RrepMsg(*this); } };
inline unsigned faceBits(size_t n) { return (unsigned)((3 * n + 7) / 8); }    // a face index is 3 bits
// split="walk": a walker goes around the gap toward a cell of the other piece (greedy, then right-hand wall following).
// 10 bytes + the faces it took (3 bits each, out and back)
struct WalkMsg : Message { int key, region, cell, gc, seq, ttl, follow = 0, heading = 0, hitCell = -1, hitDist = 0, steps = 0, hitHeading = -1, hand = 1; bID origin;
    std::vector<int> out, back;
    WalkMsg(int k, int r, int c, int g, bID o, int s, int t) : key(k), region(r), cell(c), gc(g), seq(s), ttl(t), origin(o) { type = WALK_ID; }
    unsigned int size() const override { return 10 + faceBits(out.size()) + faceBits(back.size()); } Message *clone() const override { return new WalkMsg(*this); } };
// the answer: the piece key and the way (out faces) for the one who asked; travels back on the back faces. 5 bytes + faces
struct WalkRepMsg : Message { int key, want = -1; bID origin; std::vector<int> out, back;   // want: the name the asker used (+2 B)
    WalkRepMsg(int k, bID o) : key(k), origin(o) { type = WALKREP_ID; }
    unsigned int size() const override { return 7 + faceBits(out.size()) + faceBits(back.size()); } Message *clone() const override { return new WalkRepMsg(*this); } };
// inside a piece: "the gateway to piece key is at cell gw", 4 bytes, flooded once in the piece
struct GwMsg : Message { int key, gw;
    GwMsg(int k, int g) : key(k), gw(g) { type = GW_ID; }
    unsigned int size() const override { return 4; } Message *clone() const override { return new GwMsg(*this); } };

// ADV(r, D), 3 bytes: [region 3 bits | sender's region 3 bits | 2 flag bits] [flag bit + spare] [distance 1 B, < dmax <= 255]
//   flag 1: "you are my parent in the map wave"   flag 2 / 4: "I depend on you" on / off (replaces a SUB)
struct AdvMsg : Message { int region, dist, src = 0, flags = 0;
    AdvMsg(int r, int d, int s = 0) : region(r), dist(d), src(s) { type = ADV_ID; }
    unsigned int size() const override { return 3; } Message *clone() const override { return new AdvMsg(*this); } };
struct SubMsg : Message { bool on;
    SubMsg(bool o) : on(o) { type = SUB_ID; }
    unsigned int size() const override { return 1; } Message *clone() const override { return new SubMsg(*this); } };
// HEARTBEAT, 1 byte: [load level 3 bits | energy level 3 bits | "ahead byte follows" | "ahead unchanged"]
//   (+1 byte: congested-ahead mask, only when it changed since this neighbour last heard it and is not empty)
struct LinkRep { bool rep = false; int hBase = 0, rHi = 0; unsigned hMask = 0, rMask = 0;   // linkAck report: passed on (all < hBase + mask), received (all <= rHi + mask)
    unsigned bytes() const { return rep ? 3 + (hMask ? 4 : 0) + (rMask ? 4 : 0) : 0; } };
struct HbMsg : Message, LinkRep { int load = 0, energy = 0, ahead = 0, advSum = -1; bool levels = false, keepAhead = false;
    HbMsg() { type = HB_ID; }
    // report: received-up-to (2 B) + how far "passed on" lags behind it (1 B) + each 32-bit mask only when not empty
    unsigned int size() const override { return ((ahead && !keepAhead) ? 2 : 1) + bytes() + (advSum >= 0 ? 1 : 0); } Message *clone() const override { return new HbMsg(*this); } };   // advSync: + 1-byte checksum of the distances I advertise on this link
// STATE, 2 bytes: [overload level 3 bits | energy deficit level 3 bits | spare] [congested-ahead mask, 1 bit per region]
// levels count only what is past the threshold (0 = healthy)
// sent only when one of the levels changes (load: busiest outgoing link; energy: battery)
struct StateMsg : Message { int load, energy, ahead;
    StateMsg(int l, int e, int a) : load(l), energy(e), ahead(a) { type = STATE_ID; }
    unsigned int size() const override { return 2; } Message *clone() const override { return new StateMsg(*this); } };
// BYE, 1 byte: "my battery is almost empty, I stop now" (a predictable failure, announced instead of detected)
struct ByeMsg : Message { ByeMsg() { type = BYE_ID; }
    unsigned int size() const override { return 1; } Message *clone() const override { return new ByeMsg(*this); } };
struct MapTreeMsg : Message { int wave = -1;                         // elect: the initiator id (+2 B)
    MapTreeMsg(int w = -1) : wave(w) { type = MTREE_ID; }
    unsigned int size() const override { return wave >= 0 ? 3 : 1; } Message *clone() const override { return new MapTreeMsg(*this); } };
struct MapBoxesMsg : Message { rmap::Bytes boxes; int score = -1;     // adapt: the region's geo score (1 B, map broadcast only)
    int wave = -1;                                                       // elect: wave of an echo (+2 B)
    MapBoxesMsg(int t, const rmap::Bytes &b, int sc = -1, int w = -1) : boxes(b), score(sc), wave(w) { type = t; }
    unsigned int size() const override { return 1 + boxes.size() + (score >= 0 ? 1 : 0) + (wave >= 0 ? 2 : 0); } Message *clone() const override { return new MapBoxesMsg(*this); } };
struct MapDelMsg : Message { std::vector<int> cells; int ver = 0;     // "these cells are dead": 2 B each (+1 B count when several); ver > 0 only after a re-add (+1 B)
    MapDelMsg(int c) : cells{c} { type = MDEL_ID; }
    MapDelMsg(int c, int v) : cells{c}, ver(v) { type = MDEL_ID; }
    MapDelMsg(std::vector<int> c) : cells(std::move(c)) { type = MDEL_ID; }
    unsigned int size() const override { return (cells.size() == 1 ? 2 : 1 + 2 * cells.size()) + (ver ? 1 : 0); } Message *clone() const override { return new MapDelMsg(*this); } };
struct DvMsg : Message { int cell, dist;
    DvMsg(int c, int d) : cell(c), dist(d) { type = DV_ID; }
    unsigned int size() const override { return 3; } Message *clone() const override { return new DvMsg(*this); } };
// data packet: 16-byte header as before (+2 target cell, +2 per waypoint still to follow)
struct UDataMsg : Message, LinkRep {
    int pid; bID src; int srcRegion, dstRegion, dstCell; int hops, entryHops;
    std::vector<int> wp; int wpi = 0;
    int recov = -1;                         // mapgeo: distance where greedy got stuck (recovery mode), -1 = greedy
    int how = -1;                           // adapt: chosen by the entry cube, 0 = geo, 1 = planned route (1 bit)
    int gc = 0;                             // delivery mode GEOCAST (a header field, no extra byte)
    int det = -1;                           // balance="detour": sideways hops still allowed (2 spare header bits), -1 = off
    int salv = 0;
    int swait = 0;                          // split: waiting for the way to another piece (longer wait)
    int sub = -1;                           // split: the piece (region, piece cell) this packet is being led to (+2 B while set)
    std::vector<int> sr; int sri = 0;       // walk: the gateway's way to the other piece (3 bits per face still to take)
    int via = -1, realCell = -1;            // walk: inside my piece, heading to the gateway cell first (+2 B: the real target)
    int fan = 0;                            // split geocast: this copy is for one piece only (1 bit)
    int ack = -1, gone = 0, srcCell = -1, ackKey = -1;
    int gpLp = -1, gpE0a = -1, gpE0b = -1; double gpLf = 1e18;   // GPSR perimeter mode: where it got stuck, first edge of the face, last face-change distance (+6 B)   // ackKey: the key the receipt answers (geocast piece copies: one per piece)
    int lseq = -1;                          // linkAck: sequence number on this link (+2 B)
    int inFace = -1, inSeq = -1, parked = 0, lrecv = 0, lres = 0;   // local bookkeeping, not sent   // e2e: this message acknowledges packet "ack" (gone: its target is dead); srcCell: where to send the ack (+2 B)
    int esc = 0, escLeft = 0;                            // v5: escape mode, walking around a cut toward the target cell through any module (1 bit)                           // v5: this copy was sent again by the recovery buffer (1 spare header bit)
    UDataMsg(int p, bID s, int sr, int dr, int dc) : pid(p), src(s), srcRegion(sr), dstRegion(dr), dstCell(dc), hops(0), entryHops(-1) { type = UDATA_ID; }
    unsigned int size() const override { if (ack >= 0) return 8 + 2 + 1 + (lseq >= 0 ? 2 : 0) + bytes(); return bytes() + (lseq >= 0 ? 2 : 0) + (srcCell >= 0 ? 2 : 0) + 16 + (dstCell >= 0 ? 2 : 0) + 2 * (unsigned)(wp.size() - wpi) + (recov >= 0 ? 2 : 0) + (sub >= 0 ? 2 : 0) + (sr.empty() ? 0 : 1 + faceBits(sr.size() - sri)) + (via >= 0 ? 2 : 0); }   // det: 2 spare bits of the 16-byte header
    Message *clone() const override { return new UDataMsg(*this); }
};

class BbRDRM5Code : public BlinkyBlocksBlockCode {
public:
    static const int MAXF = 6;
    static const int INF = 1 << 20;
private:
    BlinkyBlocksBlock *module = nullptr;
    int region = 0; bool isSource = false, willFail = false, isLeader = false, isRootCfg = false;
    bool gpsrStep(UDataMsg *d, int fromFace);
    rmap::C3 myCell{0, 0, 0}; int myIdx = 0;
    int faceCell[MAXF];                      // cell index of the neighbour on each face (-1 none)
    int dist[r2::NB_REGIONS + 1];
    int nbDist[MAXF][r2::NB_REGIONS + 1];
    // heartbeats
    int deadMask = 0, subMask = 0, childMask = 0;
    Time lastHeard[MAXF];
    Time lastAdvIn[MAXF] = {0}, lastAdvReq[MAXF] = {0}; int hbNo[MAXF] = {0};   // advSync
    int advHash(int f, bool mine) const;
    Time lastDataOut[MAXF] = {0}, fastSince[MAXF] = {0};   // hbAdapt
    std::vector<std::pair<int, int>> delPending; bool delTimer = false;   // mapDelBatch: (cell, face it came from) waiting to be passed on
    void flushDel();
    int falseMask = 0; std::set<int> deletedNb;            // faces that gave a false alarm; neighbour cells I erased from the map
    void mapAdd(int c, int fromFace, int ver = -1);
    std::map<int, int> cellVer; std::map<int, Time> cellDelT;   // when a cell was erased from my map                              // falseFix: version of the news about each cell (newer news wins)
    int txSeq[MAXF] = {0}, rHi[MAXF] = {0}, hBase[MAXF] = {1, 1, 1, 1, 1, 1}; std::set<int> rSet[MAXF], hSet[MAXF]; bool repDirty[MAXF] = {false};
    void linkHandled(const UDataMsg &d);
    void linkReport(int f, const LinkRep *h);
    void fillRep(int f, LinkRep *r);
    void linkSend(UDataMsg &d, int f);
    std::map<int, int> maxMiss; std::set<int> deadEnds;   // pieces (lowest cell) not heard even at full reach: out of reach
    bool deadEnd(UDataMsg *d);
    bool fastWatch(int f) const;
    Time hbBusyW() const { return r2::cfg.hbScale == "on" ? std::max(r2::cfg.hbBusy, 4 * r2::cfg.hbPeriod) : r2::cfg.hbBusy; }
    Time hbSlowP() const { return r2::cfg.hbScale == "on" ? std::max(r2::cfg.hbSlow, 2 * r2::cfg.hbPeriod) : r2::cfg.hbSlow; }
    Time watchFrom(int f) const;
    Time lastSent[MAXF];
    int faceRegion[MAXF];                    // neighbour's region (merged mode: learned from its ADVs)
    int sentSub = 0;                         // what my parents have been told (subMask ^ sentSub = to tell)
    bool joined = false; int doneMask = 0, echoGot = 0;   // merged map wave
    rmap::BBox bbox; int bboxVer = -1;
    Time holdUntil[r2::NB_REGIONS + 1];
    int wave = 1 << 30;                      // elect: smallest wave joined
    double geoScore = -1;                    // adapt: share of box-to-box trips a straight greedy walk completes
    // region map
    rmap::Bytes regionMap; bool mapReady = false; int mapVersion = 0;
    bool echoStarted = false, echoSent = false; int echoParent = -1, pending = 0, mapChildren = 0;
    rmap::Bytes subtree;
    // dv
    std::map<int, int> dvDist;
    std::map<int, std::array<int, MAXF>> dvNb;
    // flood
    std::set<int> seen;
    // route cache: target -> (map version, waypoints)
    std::list<std::pair<int, std::pair<int, std::vector<int>>>> cache;
    long peakScratch = 0;
    // v3: load and energy state
    double txUs[MAXF];                       // transmission time queued on each face during the current tick (us)
    double linkU[MAXF];                      // smoothed utilisation of each outgoing link (0..1)
    int myLoad = 0, myEnergy = 0, myAhead = 0;           // my levels (0..7) and congested-ahead mask
    int sentLoad = 0, sentEnergy = 0, sentAhead = 0;   // neighbours assume healthy until told  // what my neighbours were told
    int nbLoad[MAXF], nbEnergy[MAXF], nbAhead[MAXF];     // what my neighbours told me
    int sticky[r2::NB_REGIONS + 1];          // current next hop per region (aware: kept until it is worth switching)
    unsigned rrCount[r2::NB_REGIONS + 1];
    double battery = 0;
    // v4: levels inside the heartbeats
    int toldLoad[MAXF], toldEnergy[MAXF], toldAhead[MAXF];   // what each neighbour last heard from me
    Time lastDataFrom[MAXF];                 // last data packet received on each face (users)
    bool known[MAXF];                        // I have heard this neighbour's levels at least once
    void sendHb(int f, bool early);
    // v5: recovery buffer
    std::deque<std::pair<Time, UDataMsg>> sentLog[MAXF];    // copies of the data packets sent on each face, newest last
    std::deque<std::pair<Time, UDataMsg>> park;             // packets with no route right now, waiting (time parked)
    Time myFailAt = 0;
    bool parkTimer = false, reparking = false; Time curParkT = 0;   // reparking: route() called from flushPark
    struct Pend { UDataMsg msg; Time due; int tries; int gone = 0; Time unreachT = 0; };
    std::map<int, Pend> pend;                               // e2e source: packets not acknowledged yet (pid -> original, next resend)
    bool retxTimer = false; Time retxAt = 0;
    double srtt = -1, rttvar = 0;                           // rtoAdapt: smoothed round-trip time of my receipts (first tries only)
    Time curRto() const;
    void armRetx();
    void fanOut(UDataMsg &g, int targetCell, int k);
    void sendAck(const UDataMsg *d, int gone);
    std::unordered_set<int> got;                            // destination: packets already delivered here (duplicates die)
    long bufB = 0, peakBufB = 0;                            // bytes held by the two buffers now / at most
    void logSent(int f, const UDataMsg &d);
    void salvageFace(int f);
    void parkOrDrop(UDataMsg *d);
    void escapeStep(UDataMsg *d, int fromFace);
    bool cutPiece = false; int extSent = -1;            // v5: I am in a piece of my region cut off from its main part
    void computePiece();
    // split regions
    int pieceVer = -1, nPieces = 1, myLow = -1; std::vector<int> pcomp, plow;
    void pieceInfo();
    int pieceOf(int cell);                               // piece cell (lowest cell) of the piece holding cell, -1 = region whole / not in map
    bool splitTimer = false; int seedKey = -1;
    struct SubG { int d = 1 << 20; int nb[6] = {1 << 20, 1 << 20, 1 << 20, 1 << 20, 1 << 20, 1 << 20}; Time hold = 0; bool seed = false; int lim = 0; };
    std::map<int, SubG> sub;                             // grad: piece key -> my distance, neighbours' distances
    void subRecalc(int key, bool force = false);
    int regLim[8] = {0}; bool widenTimer = false, wantWiden = false;
    void needWiden();
    void widenTo(int r, int lim, int fromFace);
    struct SRoute { int face, hops; };
    std::map<int, SRoute> routes;                        // seek: piece key -> next hop
    std::map<long, std::pair<int, Time>> rreqRev;        // seek: question (origin, seq) -> face it came from
    std::map<int, std::pair<Time, int>> seekPending;     // seek: (region, piece cell in my view) -> (deadline, ttl)
    struct SeekInfo { int cell, gc, tries; };
    std::map<int, SeekInfo> seekInfo;                    // seek: pending question -> cell asked for, geocast?, tries so far
    int seekSeq = 0;
    std::map<int, Time> floodFailed;                     // walk: a flooded question for this cell went unanswered at
    std::map<int, Time> seekFailed;                      // seek: question -> when it last went unanswered
    void startSeek(int pk, int cell, int ttl, int gc, int tries = 1);
    int findKey(int region, int pieceLow, int cell, int gc);
    int splitFace(UDataMsg *d);
    struct WRoute { std::vector<int> path; int gw = -1; };
    std::map<int, WRoute> wroutes;                       // walk: piece key -> my way there (gateway) or my piece's gateway cell
    std::map<int, int> walkFails;                        // walk: unanswered walks per question
    std::map<long, size_t> walkSeen;                     // walk: (origin, seq) -> length of the way when it first passed here (loops cut)
    void startWalk(int pk, int cell, int gc, int key, int tries = 1);
    int nearestCellOf(int pieceLow);                     // walk: the cell of that piece (in my map) closest to me
    std::deque<std::pair<Time, UDataMsg>> gcLog;         // geocast entry: recent geocasts, resent into pieces found later
    int lastPieces = 1;
    void walkStep(WalkMsg *w, int fromFace);
    bool walkForward(UDataMsg *d, int fromFace);
    bool inPiece(int key);
    int extDist() const;
    int advDist(int r, int f) const;
    void flushPark();
    bool suspected(int f) const;
    void spend(double uJ);
    double penalty(int f, int r) const;
    int chooseFace(int r, UDataMsg *d, int fromFace);
    void stateTick();
    void geocastStep(UDataMsg *d, int fromFace);

    bool usable(int f) const;
    int inRegionMask() const;
    int bestFace(int r) const;
    void announce(int r);
    bool recalc(int r);
    void recomputeAll();
    void updateSubs();
    void flushSubs();
    void mergedCheck();
    const rmap::BBox &box();
    void heard(int f);
    void declareDead(int f);
    void echoStart(int parent);
    void markDead(int f);
    void startWave(int w, int parent);
    void checkEcho();
    void installMap(const rmap::Bytes &b, int score = -1);
    void mapDelete(int cell, int fromFace, int ver = -1);
    void dvRecompute(int c);
    int faceFor(int cell) const;
    void sendData(UDataMsg *d, int f);
    void route(UDataMsg *d, int fromFace);
    bool inRegionStep(UDataMsg *d, int fromFace);
    template <class M> void sendCtl(int f, M *msg, r2::Kind k);

public:
    BbRDRM5Code(BlinkyBlocksBlock *host);
    ~BbRDRM5Code() {};
    void startup() override;
    void parseUserElements(TiXmlDocument *config) override;
    void parseUserBlockElements(TiXmlElement *config) override;
    void onInterruptionEvent(std::shared_ptr<Event> event) override;
    void onNeighborChanged(uint64_t face, int action) override;
    void onMsg(std::shared_ptr<Message> m, P2PNetworkInterface *sender);
    void onEndOfSimulation() override;
    std::string onInterfaceDraw() override;
    long memBytes(long &grad, long &hb, long &inreg, long &scratch) const;

    static BlockCode *buildNewBlockCode(BuildingBlock *host) { return new BbRDRM5Code((BlinkyBlocksBlock *)host); }
};

#endif
