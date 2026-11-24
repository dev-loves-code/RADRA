#!/usr/bin/env python3
"""
3D configs for bbRDRM5 (Blinky Blocks, 6 faces).

Usage: make_rdrm3d.py out.xml name k shape [key=value ...]
  shape   h3       humanoid with depth: torso and head 2k deep, limbs k deep
          tunnels  h3 with 4 square tunnels through the torso (front to back) and one through the head
          cavity   h3 with closed cavities inside the torso and the head (holes you cannot see from outside)
          maze3    h3 whose torso is a 3D maze: walls across it, openings alternating left/right AND front/back
          swiss3:pct:seed  h3 with pct % of the cells missing at random (every region stays in one piece)
          cube     a solid 3D block cut into 6 regions (2 layers x 3 columns)
  keys    as make_rdrm2.py: fail=none|random:pct:seed|slab:axis:value (a whole slab of the robot dies)
          failAt, failSpread, spreadSeed, any <experiment> attribute
"""
import sys, random
from collections import deque

COLORS = {1: "240,200,60", 2: "70,130,220", 3: "60,180,90", 4: "40,150,150", 5: "220,90,70", 6: "170,80,190"}

def h3(k):
    cells = {}
    def box(x0, x1, y0, y1, z0, z1, r):
        for x in range(x0, x1):
            for y in range(y0, y1):
                for z in range(z0, z1):
                    assert (x, y, z) not in cells
                    cells[(x, y, z)] = r
    D = 2 * k                                   # torso / head depth
    yl0, yl1 = k // 2, k // 2 + k               # limbs: k deep, centred on the torso
    box(2*k, 4*k, yl0, yl1, 0, 6*k, 5)          # legs
    box(6*k, 8*k, yl0, yl1, 0, 6*k, 6)
    box(2*k, 8*k, 0, D, 6*k, 13*k, 2)           # torso
    box(4*k, 6*k, yl0, yl1, 13*k, 14*k, 2)      # neck
    box(3*k, 7*k, 0, D, 14*k, 18*k, 1)          # head
    box(0, 2*k, yl0, yl1, 12*k, 13*k, 3)        # left shoulder bar + arm
    box(0, k, yl0, yl1, 6*k, 12*k, 3)
    box(8*k, 10*k, yl0, yl1, 12*k, 13*k, 4)     # right
    box(9*k, 10*k, yl0, yl1, 6*k, 12*k, 4)
    return cells, 10*k, D, 18*k

def cube(k):
    n = 3 * k; cells = {}
    for x in range(n):
        for y in range(n):
            for z in range(n):
                cells[(x, y, z)] = 1 + (x * 3 // n) + 3 * (z * 2 // n)
    return cells, n, n, n

def nb(p):
    x, y, z = p
    return [(x+1, y, z), (x-1, y, z), (x, y+1, z), (x, y-1, z), (x, y, z+1), (x, y, z-1)]

def bfs(cells, s):
    d = {s: 0}; q = deque([s])
    while q:
        u = q.popleft()
        for v in nb(u):
            if v in cells and v not in d: d[v] = d[u] + 1; q.append(v)
    return d

def roots(cells):
    out = {}
    for r in range(1, 7):
        pts = [p for p, rr in cells.items() if rr == r]
        c = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
        out[r] = min(pts, key=lambda p: (sum((p[i] - c[i]) ** 2 for i in range(3)), p))
    return out

def connected(cells, r=None):
    pts = [p for p in cells if r is None or cells[p] == r]
    sub = {p: 1 for p in pts}
    return len(bfs(sub, pts[0])) == len(pts)

def shape(k, kind):
    if kind == "cube": return cube(k)
    cells, W, D, H = h3(k)
    rm = set()
    tx0, tz0 = 2 * k, 6 * k
    if kind == "tunnels":                       # front-to-back square tunnels (k x k) through the torso and the head
        for (ux, uz) in [(1, 1.5), (4, 1.5), (1, 4), (4, 4)]:
            x0, z0 = tx0 + int(ux * k), tz0 + int(uz * k)
            rm |= {(x, y, z) for x in range(x0, x0 + k) for y in range(D) for z in range(z0, z0 + k)}
        hx0, hz0 = 3 * k + int(1.5 * k), 14 * k + int(1.5 * k)
        rm |= {(x, y, z) for x in range(hx0, hx0 + k) for y in range(D) for z in range(hz0, hz0 + k)}
    elif kind == "cavity":                      # closed voids: a skin of modules all around them
        for (x0, z0, s) in [(tx0 + 1, tz0 + 1, 2 * k - 1), (tx0 + 3 * k, tz0 + 3 * k, 2 * k), (3 * k + 1, 14 * k + 1, 2 * k - 1)]:
            rm |= {(x, y, z) for x in range(x0, x0 + s) for y in range(1, D - 1) for z in range(z0, z0 + s)}
    elif kind == "maze3":                       # walls across the torso; the gap alternates left/right and front/back
        for j, uz in enumerate([1.5, 3, 4.5]):
            z = tz0 + int(uz * k)
            gx = range(tx0 + 5 * k, tx0 + 6 * k) if j % 2 == 0 else range(tx0, tx0 + k)
            gy = range(D - 1, D) if j % 2 == 0 else range(0, 1)
            rm |= {(x, y, z) for x in range(tx0, tx0 + 6 * k) for y in range(D) if not (x in gx and y in gy)}
    elif kind.startswith("swiss3"):
        _, pct, seed = kind.split(":")
        rnd = random.Random(int(seed)); rt = roots(cells)
        cand = [p for p in sorted(cells) if p not in rt.values()]; rnd.shuffle(cand)
        goal = round(len(cells) * float(pct) / 100)
        for p in cand:
            if len(rm) >= goal: break
            r = cells.pop(p)
            if connected(cells, r) and connected(cells): rm.add(p)
            else: cells[p] = r
        return cells, W, D, H
    for p in rm: cells.pop(p, None)
    return cells, W, D, H

def main():
    out, name, k, kind = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
    kv = dict(a.split("=", 1) for a in sys.argv[5:])
    cells, W, D, H = shape(k, kind)
    order = sorted(cells, key=lambda p: (p[2], p[1], p[0]))
    assert connected(cells), "shape is not connected"
    for r in range(1, 7): assert connected(cells, r), f"region {r} is not connected"
    rt = roots(cells)
    diam = max(max(bfs(cells, p).values()) for p in order[:: max(1, len(order) // 40)] + list(rt.values()))
    dmax = max(W + D + H, diam + 4)
    start = int(kv.pop("start", 5000000))
    fail = kv.pop("fail", "none")
    failAt = int(kv.pop("failAt", start + 10000000))
    spread = int(kv.pop("failSpread", 0))
    srnd = random.Random(int(kv.pop("spreadSeed", 1)))
    F = set()
    for part in fail.split("+"):                # several kinds at once: wall+random:3:1
        if part.startswith("random"):
            _, pct, seed = part.split(":")
            F |= set(random.Random(int(seed)).sample(order, round(len(order) * float(pct) / 100)))
        elif part == "wall":                    # torso + neck column at x = 5k, all depths: the body splits left / right, joined only through the head
            F |= {p for p in cells if cells[p] == 2 and p[0] == 5 * k}
        elif part.startswith("slab"):           # slab:axis:value - every module with that coordinate dies (cuts the robot)
            _, ax, v = part.split(":"); a = "xyz".index(ax)
            F |= {p for p in cells if p[a] == int(v)}
        elif part.startswith("cells:"):
            F |= {tuple(int(v) for v in c.split(",")) for c in part[6:].split(";")}
    F = {p for p in F if p in cells}
    exp = {"name": name, "start": start, "dmax": dmax}; exp.update(kv)
    if F: exp["failAt"] = failAt
    attrs = " ".join(f'{a}="{v}"' for a, v in exp.items())
    bs = 40; cx, cy, cz = W * bs / 2, D * bs / 2, H * bs / 2; dist = int(max(W, H) * bs * 1.6)
    L = ['<?xml version="1.0" standalone="no" ?>',
         f'<!-- bbRDRM5 3D: k={k} shape={kind}: {len(cells)} modules, diameter {diam}, failures {len(F)}. make_rdrm3d.py -->',
         '<vs>', '  <visuals>', '    <window size="1600x1000" backgroundColor="0xffffff" gradientColor="0xe8e8f0"/>',
         '    <render shadows="false" grid="false"/>', '  </visuals>',
         f'  <experiment {attrs}/>', f'  <world gridSize="{W},{D},{H}">',
         f'    <camera target="{cx:.0f},{cy:.0f},{cz:.0f}" thetaPhiDist="30,20,{dist}" fov="50" near="10" far="{dist*3}"/>',
         f'    <spotlight target="{cx:.0f},{cy:.0f},{cz:.0f}" thetaPhiDist="-70,40,{dist*2}" fov="40" show="0" near="10" far="{dist*4}"/>',
         '    <blockList color="200,200,200">']
    for p in order:
        r = cells[p]; extra = ""
        if rt[r] == p: extra += ' root="true"'
        if p in F:
            extra += ' fail="true"'
            if spread: extra += f' failTime="{failAt + srnd.randrange(0, spread + 1, 10000)}"'
        L.append(f'      <block position="{p[0]},{p[1]},{p[2]}" region="{r}" color="{COLORS[r]}"{extra}/>')
    L += ['    </blockList>', '  </world>', '</vs>']
    open(out, "w").write("\n".join(L) + "\n")
    print(f"{out}: N={len(cells)} diam={diam} dmax={dmax} fail={len(F)}")

if __name__ == "__main__":
    main()
