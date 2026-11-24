/**
 * @file rdrm2Map.hpp
 * The region map: a tiny encoding of the region's shape, reused from our
 * sliding-cubes "lean" version (sc_reconf_lean, scReconfBits.hpp).
 *
 * STORED form   : packed boxes, 4 bytes each (low corner cell index 2 B + high corner 2 B).
 *                 A box is a block of the region with no gap in it ("clean space");
 *                 holes and gaps are simply the cells that no box covers.
 * WORKING form  : bit matrices of the region's BOUNDING BOX, 1 bit per cell, used as scratch while
 *                 merging maps or planning a path (2 bits per cell = 2 matrices).
 *
 *   paint(boxes)   : boxes -> bits
 *   encode(bits)   : bits  -> fewest boxes (greedy, best of 6 axis orders)
 *   nextHop        : layered BFS from the TARGET over the map (2 bits per cell, the layer
 *                    number is the distance); the first of my neighbour cells it reaches
 *                    lies on a shortest path inside the region.
 *   planRoute      : the same BFS keeping one byte per bbox cell (used once by the entry cube
 *                    to plan a whole route and compress it into waypoints).
 *
 * author: Nawar ELZEIN
 **/

#ifndef RDRM2_MAP_HPP
#define RDRM2_MAP_HPP

#include <vector>
#include <cstdint>
#include <cstring>
#include <array>
#include <algorithm>
#include <cstdlib>

namespace rmap {

typedef std::vector<uint8_t> Bytes;
typedef std::array<int, 3> C3;

inline C3 &G() { static C3 g{0, 0, 0}; return g; }        // grid size, known by every cube
inline int N() { return G()[0] * G()[1] * G()[2]; }
inline int nbytes() { return (N() + 7) / 8; }
inline int idx(const C3 &c) { return c[0] + G()[0] * (c[1] + G()[1] * c[2]); }
inline C3 cell(int i) { int X = G()[0], Y = G()[1]; return C3{i % X, (i / X) % Y, i / (X * Y)}; }
inline bool inGrid(const C3 &c) {
    return c[0] >= 0 && c[1] >= 0 && c[2] >= 0 && c[0] < G()[0] && c[1] < G()[1] && c[2] < G()[2];
}
static const int ND[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};

inline bool get(const Bytes &v, int i) { return (v[i >> 3] >> (i & 7)) & 1; }
inline void set(Bytes &v, int i)       { v[i >> 3] |= (1 << (i & 7)); }
inline void clr(Bytes &v, int i)       { v[i >> 3] &= ~(1 << (i & 7)); }
inline Bytes zeros()                    { return Bytes(nbytes(), 0); }

// ---- packed boxes
struct Box { int lo, hi; };
inline int boxCount(const Bytes &p) { return (int)p.size() / 4; }
inline Box boxAt(const Bytes &p, int k) {
    return Box{ p[4*k] | (p[4*k+1] << 8), p[4*k+2] | (p[4*k+3] << 8) };
}
inline bool inBox(const Box &b, const C3 &c) {
    C3 l = cell(b.lo), h = cell(b.hi);
    for (int a = 0; a < 3; a++) if (c[a] < l[a] || c[a] > h[a]) return false;
    return true;
}
inline int boxOf(const Bytes &p, const C3 &c) {             // which box holds c (-1 = none)
    for (int k = 0; k < boxCount(p); k++) if (inBox(boxAt(p, k), c)) return k;
    return -1;
}
inline bool has(const Bytes &p, const C3 &c) { return boxOf(p, c) >= 0; }
inline Bytes single(const C3 &c) {
    int i = idx(c);
    return { (uint8_t)(i & 0xFF), (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), (uint8_t)(i >> 8) };
}

inline void paint(Bytes &v, const Bytes &p) {
    for (int k = 0; k < boxCount(p); k++) {
        Box b = boxAt(p, k); C3 l = cell(b.lo), h = cell(b.hi);
        for (int z = l[2]; z <= h[2]; z++) for (int y = l[1]; y <= h[1]; y++)
            for (int x = l[0]; x <= h[0]; x++) set(v, idx(C3{x, y, z}));
    }
}

inline bool full(const Bytes &v, const int lo[3], const int hi[3]) {
    for (int x = lo[0]; x <= hi[0]; x++) for (int y = lo[1]; y <= hi[1]; y++) for (int z = lo[2]; z <= hi[2]; z++) {
        if (x >= G()[0] || y >= G()[1] || z >= G()[2]) return false;
        if (!get(v, idx(C3{x, y, z}))) return false;
    }
    return true;
}

// one greedy pass in axis order (o0,o1,o2) that eats the bits of w (same rule as sc_reconf_lean)
inline int encodeOrder(Bytes &w, int o0, int o1, int o2, Bytes *out) {
    int n = 0, c[3];
    for (c[o2] = 0; c[o2] < G()[o2]; c[o2]++) for (c[o1] = 0; c[o1] < G()[o1]; c[o1]++)
        for (c[o0] = 0; c[o0] < G()[o0]; c[o0]++) {
            if (!get(w, idx(C3{c[0], c[1], c[2]}))) continue;
            int lo[3] = {c[0], c[1], c[2]}, hi[3] = {c[0], c[1], c[2]};
            for (int axis : {o0, o1, o2})
                while (true) {
                    int l[3] = {lo[0], lo[1], lo[2]}, h[3] = {hi[0], hi[1], hi[2]};
                    l[axis] = h[axis] = hi[axis] + 1;
                    if (!full(w, l, h)) break;
                    hi[axis]++;
                }
            for (int x = lo[0]; x <= hi[0]; x++) for (int y = lo[1]; y <= hi[1]; y++)
                for (int z = lo[2]; z <= hi[2]; z++) clr(w, idx(C3{x, y, z}));
            n++;
            if (out) {
                int a = idx(C3{lo[0], lo[1], lo[2]}), b = idx(C3{hi[0], hi[1], hi[2]});
                out->push_back(a & 0xFF); out->push_back(a >> 8); out->push_back(b & 0xFF); out->push_back(b >> 8);
            }
        }
    return n;
}
inline Bytes encode(const Bytes &src) {
    static const int orders[6][3] = {{0,1,2},{1,0,2},{0,2,1},{2,0,1},{1,2,0},{2,1,0}};
    int best = -1, bestCount = 0;
    for (int k = 0; k < 6; k++) {
        Bytes w = src;
        int n = encodeOrder(w, orders[k][0], orders[k][1], orders[k][2], nullptr);
        if (best < 0 || n < bestCount) { best = k; bestCount = n; }
    }
    Bytes out, w = src;
    encodeOrder(w, orders[best][0], orders[best][1], orders[best][2], &out);
    return out;
}
inline Bytes unite(const Bytes &P, const Bytes &Q) { Bytes v = zeros(); paint(v, P); paint(v, Q); return encode(v); }
inline Bytes removeCell(const Bytes &P, const C3 &c) { Bytes v = zeros(); paint(v, P); clr(v, idx(c)); return encode(v); }

// ---- computation counters (statistics only): what a cube's processor actually does
struct Work { long paint = 0, expand = 0, boxTests = 0; };

// box tests with a counter: "is this cell in the clean space?" costs one test per box scanned
inline int boxOfW(const Bytes &p, const C3 &c, Work &w) {
    for (int k = 0; k < boxCount(p); k++) { w.boxTests++; if (inBox(boxAt(p, k), c)) return k; }
    return -1;
}
inline bool hasW(const Bytes &p, const C3 &c, Work &w) { return boxOfW(p, c, w) >= 0; }

// ---- planning works on the region's BOUNDING BOX only (scratch = bbox cells, not the grid)
struct BBox { C3 lo{0, 0, 0}, hi{-1, -1, -1};
    int dx() const { return hi[0] - lo[0] + 1; } int dy() const { return hi[1] - lo[1] + 1; } int dz() const { return hi[2] - lo[2] + 1; }
    int n() const { return hi[0] < lo[0] ? 0 : dx() * dy() * dz(); }
    bool in(const C3 &c) const { for (int a = 0; a < 3; a++) if (c[a] < lo[a] || c[a] > hi[a]) return false; return true; }
    int li(const C3 &c) const { return (c[0] - lo[0]) + dx() * ((c[1] - lo[1]) + dy() * (c[2] - lo[2])); }
    C3 cellOf(int i) const { return C3{lo[0] + i % dx(), lo[1] + (i / dx()) % dy(), lo[2] + i / (dx() * dy())}; }
};
inline BBox bboxOf(const Bytes &p) {
    BBox b; if (boxCount(p) == 0) return b;
    b.lo = C3{1 << 20, 1 << 20, 1 << 20}; b.hi = C3{-1, -1, -1};
    for (int k = 0; k < boxCount(p); k++) { Box x = boxAt(p, k); C3 l = cell(x.lo), h = cell(x.hi);
        for (int a = 0; a < 3; a++) { b.lo[a] = std::min(b.lo[a], l[a]); b.hi[a] = std::max(b.hi[a], h[a]); } }
    return b;
}
inline int bitsBytes(const BBox &b) { return (b.n() + 7) / 8; }
inline void paintB(Bytes &v, const Bytes &p, const BBox &bb, Work &w) {
    for (int k = 0; k < boxCount(p); k++) {
        Box b = boxAt(p, k); C3 l = cell(b.lo), h = cell(b.hi);
        for (int z = l[2]; z <= h[2]; z++) for (int y = l[1]; y <= h[1]; y++)
            for (int x = l[0]; x <= h[0]; x++) { set(v, bb.li(C3{x, y, z})); w.paint++; }
    }
}

// First of my neighbour cells (in 'mine' order) reached by a layered BFS from the target over the
// map (2 bits per bbox cell + 1 occupancy bit). Returns its index in 'mine', or -1.
inline int nextHop(const Bytes &map, const BBox &bb, const C3 &target, const std::vector<C3> &mine, Work &w) {
    for (size_t k = 0; k < mine.size(); k++) if (mine[k] == target) return (int)k;
    int NB = bitsBytes(bb);
    Bytes occ(NB, 0), A(NB, 0), B(NB, 0);              // 00 not reached, 01 current, 10 next, 11 done
    paintB(occ, map, bb, w);
    if (!bb.in(target)) return -1;
    set(B, bb.li(target));
    std::vector<int> mineIdx; for (auto &c : mine) mineIdx.push_back(bb.in(c) ? bb.li(c) : -1);
    while (true) {
        bool any = false; int found = -1;
        for (int i = 0; i < bb.n(); i++) {
            if (get(A, i) || !get(B, i)) continue;
            w.expand++;
            C3 c = bb.cellOf(i);
            for (auto &d : ND) {
                C3 n{c[0] + d[0], c[1] + d[1], c[2] + d[2]};
                if (!bb.in(n)) continue;
                int j = bb.li(n);
                if (!get(occ, j) || get(A, j) || get(B, j)) continue;
                set(A, j); any = true;
                for (size_t k = 0; k < mineIdx.size(); k++)
                    if (mineIdx[k] == j && (found < 0 || (int)k < found)) found = (int)k;
            }
        }
        if (found >= 0) return found;
        if (!any) return -1;
        for (int i = 0; i < bb.n(); i++) {
            bool a = get(A, i), b = get(B, i);
            if (!a && b) set(A, i); else if (a && !b) { clr(A, i); set(B, i); }
        }
    }
}

// full shortest path inside the map, compressed into waypoints (turn cells, then the target).
// Uses 1 byte per bbox cell while planning. Empty = unreachable.
inline std::vector<int> planRoute(const Bytes &map, const BBox &bb, const C3 &from, const C3 &target, Work &w) {
    std::vector<int> wp;
    if (!bb.in(from) || !bb.in(target)) return wp;
    Bytes occ(bitsBytes(bb), 0); paintB(occ, map, bb, w);
    std::vector<uint8_t> D(bb.n(), 255);
    std::vector<int> q; q.push_back(bb.li(target)); D[bb.li(target)] = 0;
    for (size_t h = 0; h < q.size(); h++) {
        int i = q[h]; w.expand++; C3 c = bb.cellOf(i);
        for (auto &d : ND) {
            C3 n{c[0] + d[0], c[1] + d[1], c[2] + d[2]};
            if (!bb.in(n)) continue;
            int j = bb.li(n);
            if (!get(occ, j) || D[j] != 255) continue;
            D[j] = D[i] + 1; q.push_back(j);
        }
    }
    if (D[bb.li(from)] == 255) return wp;
    C3 cur = from; int lastDir = -1;
    while (!(cur == target)) {
        int best = -1; C3 nx{0, 0, 0};
        for (int k = 0; k < 6; k++) {                    // keep going straight when possible
            int kk = (lastDir >= 0) ? (k == 0 ? lastDir : (k <= lastDir ? k - 1 : k)) : k;
            C3 n{cur[0] + ND[kk][0], cur[1] + ND[kk][1], cur[2] + ND[kk][2]};
            if (!bb.in(n)) continue;
            if (D[bb.li(n)] == D[bb.li(cur)] - 1) { best = kk; nx = n; break; }
        }
        if (best < 0) { wp.clear(); return wp; }
        if (lastDir >= 0 && best != lastDir) wp.push_back(idx(cur));
        lastDir = best; cur = nx;
    }
    wp.push_back(idx(target));
    return wp;
}

// ---- merging on the bounding box only (echo convergecast, map deletes): scratch = 2 bbox bit matrices
inline BBox bboxUnion(const BBox &a, const BBox &b) {
    if (a.n() == 0) return b; if (b.n() == 0) return a;
    BBox u; for (int k = 0; k < 3; k++) { u.lo[k] = std::min(a.lo[k], b.lo[k]); u.hi[k] = std::max(a.hi[k], b.hi[k]); } return u;
}
inline bool fullB(const Bytes &v, const BBox &bb, const int lo[3], const int hi[3]) {
    for (int x = lo[0]; x <= hi[0]; x++) for (int y = lo[1]; y <= hi[1]; y++) for (int z = lo[2]; z <= hi[2]; z++) {
        C3 c{x, y, z}; if (!bb.in(c) || !get(v, bb.li(c))) return false; }
    return true;
}
inline int encodeOrderB(Bytes &w, const BBox &bb, int o0, int o1, int o2, Bytes *out) {
    int n = 0, c[3];
    for (c[o2] = bb.lo[o2]; c[o2] <= bb.hi[o2]; c[o2]++) for (c[o1] = bb.lo[o1]; c[o1] <= bb.hi[o1]; c[o1]++)
        for (c[o0] = bb.lo[o0]; c[o0] <= bb.hi[o0]; c[o0]++) {
            if (!get(w, bb.li(C3{c[0], c[1], c[2]}))) continue;
            int lo[3] = {c[0], c[1], c[2]}, hi[3] = {c[0], c[1], c[2]};
            for (int axis : {o0, o1, o2})
                while (true) {
                    int l[3] = {lo[0], lo[1], lo[2]}, h[3] = {hi[0], hi[1], hi[2]};
                    l[axis] = h[axis] = hi[axis] + 1;
                    if (!fullB(w, bb, l, h)) break;
                    hi[axis]++;
                }
            for (int x = lo[0]; x <= hi[0]; x++) for (int y = lo[1]; y <= hi[1]; y++)
                for (int z = lo[2]; z <= hi[2]; z++) clr(w, bb.li(C3{x, y, z}));
            n++;
            if (out) {
                int a = idx(C3{lo[0], lo[1], lo[2]}), b = idx(C3{hi[0], hi[1], hi[2]});
                out->push_back(a & 0xFF); out->push_back(a >> 8); out->push_back(b & 0xFF); out->push_back(b >> 8);
            }
        }
    return n;
}
inline Bytes encodeB(const Bytes &src, const BBox &bb) {
    static const int orders[6][3] = {{0,1,2},{1,0,2},{0,2,1},{2,0,1},{1,2,0},{2,1,0}};
    int best = -1, bestCount = 0;
    for (int k = 0; k < 6; k++) { Bytes w = src; int n = encodeOrderB(w, bb, orders[k][0], orders[k][1], orders[k][2], nullptr);
        if (best < 0 || n < bestCount) { best = k; bestCount = n; } }
    Bytes out, w = src; encodeOrderB(w, bb, orders[best][0], orders[best][1], orders[best][2], &out);
    return out;
}
inline Bytes uniteB(const Bytes &P, const Bytes &Q, long &scratch) {
    BBox bb = bboxUnion(bboxOf(P), bboxOf(Q)); Work w;
    Bytes v(bitsBytes(bb), 0); paintB(v, P, bb, w); paintB(v, Q, bb, w);
    scratch = std::max<long>(scratch, 2L * bitsBytes(bb));
    return encodeB(v, bb);
}
inline Bytes removeCellB(const Bytes &P, const C3 &c, long &scratch) {
    BBox bb = bboxOf(P); Work w;
    Bytes v(bitsBytes(bb), 0); paintB(v, P, bb, w); if (bb.in(c)) clr(v, bb.li(c));
    scratch = std::max<long>(scratch, 2L * bitsBytes(bb));
    return encodeB(v, bb);
}

inline int manhattan(const C3 &a, const C3 &b) { return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]); }

} // namespace rmap

#endif
