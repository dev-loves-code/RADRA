#!/usr/bin/env python3
"""
Configs for bbRDRM2 (heartbeats, unicast, in-region navigation).

Usage: make_rdrm2.py out.xml name k shape [key=value ...]
  shape   plain | holes | maze | swiss:pct:seed (pct % of cells missing at random)
  keys    any <experiment> attribute of bbRDRM2 (traffic, inreg, detect, hbPeriod, hbMiss,
          failMode, targets, spacing, start, mapAt, cacheSize, traceDst ...) plus
          fail=none|torso|shoulder|armcut|random:pct:seed|cells:x,z;x,z   (cubes that break at failAt)
          failAt=us  (default: start + 10 s, i.e. during the traffic)
          failSpread=us (each failing module dies at a random date in [failAt, failAt + spread])
          src=x,z    (trace: only this cube sends; with traceDst)

Shapes (scale k, humanoid of make_humanoid.py):
  plain   the humanoid: every region is a few rectangles
  holes   4 square holes (k x k) in the torso and one in the head
  maze    3 one-row walls across the torso with alternating openings: the body becomes a
          serpentine corridor (many boxes, long in-region paths)
"""
import sys, random
from collections import deque
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from make_humanoid import humanoid, roots, COLORS

def shape(k, kind):
    cells, W, H = humanoid(k)
    rm = set()
    tx0, tz0 = 2 * k, 6 * k                      # torso low corner, 6k wide, 7k tall
    if kind == "holes":
        for (ux, uz) in [(1, 1.5), (4, 1.5), (1, 4), (4, 4)]:
            x0, z0 = tx0 + int(ux * k), tz0 + int(uz * k)
            rm |= {(x, z) for x in range(x0, x0 + k) for z in range(z0, z0 + k)}
        hx0, hz0 = 3 * k + int(1.5 * k), 14 * k + int(1.5 * k)
        rm |= {(x, z) for x in range(hx0, hx0 + k) for z in range(hz0, hz0 + k)}
    elif kind == "maze":
        for j, uz in enumerate([1.5, 3, 4.5]):
            z = tz0 + int(uz * k)
            xs = range(tx0, tx0 + 5 * k) if j % 2 == 0 else range(tx0 + k, tx0 + 6 * k)
            rm |= {(x, z) for x in xs}
    elif kind.startswith("swiss"):              # swiss:pct:seed - pct % of the cells missing at random,
        _, pct, seed = kind.split(":")          # every region (and the robot) stays in one piece, roots kept
        rnd = random.Random(int(seed)); rt = roots(cells)
        cand = [p for p in sorted(cells) if p not in rt.values()]
        rnd.shuffle(cand); goal = round(len(cells) * float(pct) / 100)
        for p in cand:
            if len(rm) >= goal: break
            r = cells.pop(p)
            pts = [q for q in cells if cells[q] == r]
            ok = len(bfs({q: 1 for q in pts}, pts[0])) == len(pts) and len(bfs(cells, next(iter(cells)))) == len(cells)
            if ok: rm.add(p)
            else: cells[p] = r
        return cells, W, H
    for p in rm: cells.pop(p, None)
    return cells, W, H

def neighbours(p):
    x, z = p
    return [(x + 1, z), (x - 1, z), (x, z + 1), (x, z - 1)]

def bfs(cells, s):
    d = {s: 0}; q = deque([s])
    while q:
        u = q.popleft()
        for v in neighbours(u):
            if v in cells and v not in d: d[v] = d[u] + 1; q.append(v)
    return d

def main():
    out, name, k, kind = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
    kv = dict(a.split("=", 1) for a in sys.argv[5:])
    cells, W, H = shape(k, kind)
    order = sorted(cells, key=lambda p: (p[1], p[0]))
    assert len(bfs(cells, order[0])) == len(cells), "shape is not connected"
    for r in range(1, 7):
        pts = [p for p in cells if cells[p] == r]
        assert len(bfs({p: 1 for p in pts}, pts[0])) == len(pts), f"region {r} is not connected"
    rt = roots(cells)
    diam = max(max(bfs(cells, p).values()) for p in order[:: max(1, len(order) // 40)] + [rt[1], rt[5], rt[6]])
    dmax = max(W + H, diam + 4)
    start = int(kv.pop("start", 5000000))
    fail = kv.pop("fail", "none")
    failAt = int(kv.pop("failAt", start + 10000000))
    spread = int(kv.pop("failSpread", 0))            # failures spread over [failAt, failAt + spread] us (one random date per module)
    import random as _r; srnd = _r.Random(int(kv.pop("spreadSeed", 1)))
    src = kv.pop("src", None)
    F = set()
    if fail == "torso":       # three cubes inside the torso: new holes the maps must learn
        F = {(4 * k + 1, 9 * k), (5 * k, 9 * k), (3 * k, 11 * k)}
    elif fail == "shoulder":
        F = {(2 * k, 12 * k)}
    elif fail == "armcut":    # the whole left-shoulder joint: the left arm is cut off
        F = {(2 * k, z) for z in range(12 * k, 13 * k)}
    elif fail.startswith("random"):
        _, pct, seed = fail.split(":")
        F = set(random.Random(int(seed)).sample(order, round(len(order) * float(pct) / 100)))
    elif fail.startswith("cells:"):
        F = {tuple(int(v) for v in c.split(",")) for c in fail[6:].split(";")}
    F = {p for p in F if p in cells}
    exp = {"name": name, "start": start, "dmax": dmax}
    exp.update(kv)
    if F: exp["failAt"] = failAt
    if src: exp["sources"] = "marked"; exp.setdefault("hopDelay", 150000)
    attrs = " ".join(f'{a}="{v}"' for a, v in exp.items())
    bs = 40; cx, cz = W * bs / 2, H * bs / 2; dist = int(max(W, H) * bs * 1.45)
    L = ['<?xml version="1.0" standalone="no" ?>',
         f'<!-- bbRDRM2: humanoid k={k} shape={kind}: {len(cells)} modules, diameter {diam}, failures {len(F)}. make_rdrm2.py -->',
         '<vs>', '  <visuals>', '    <window size="1600x1000" backgroundColor="0xffffff" gradientColor="0xe8e8f0"/>',
         '    <render shadows="false" grid="false"/>', '  </visuals>',
         f'  <experiment {attrs}/>', f'  <world gridSize="{W},1,{H}">',
         f'    <camera target="{cx:.0f},20,{cz:.0f}" thetaPhiDist="0,0,{dist}" fov="50" near="10" far="{dist*3}"/>',
         f'    <spotlight target="{cx:.0f},20,{cz:.0f}" thetaPhiDist="-70,20,{dist*2}" fov="40" show="0" near="10" far="{dist*4}"/>',
         '    <blockList color="200,200,200">']
    sp = tuple(int(v) for v in src.split(",")) if src else None
    for p in order:
        r = cells[p]; extra = ""
        if rt[r] == p: extra += ' root="true"'
        if p in F:
            extra += ' fail="true"'
            if spread: extra += f' failTime="{failAt + srnd.randrange(0, spread + 1, 10000)}"'   # each module dies at its own date
        if sp == p: extra += ' source="true"'
        L.append(f'      <block position="{p[0]},0,{p[1]}" region="{r}" color="{COLORS[r]}"{extra}/>')
    L += ['    </blockList>', '  </world>', '</vs>']
    open(out, "w").write("\n".join(L) + "\n")
    print(f"{out}: N={len(cells)} diam={diam} dmax={dmax} fail={len(F)} {attrs}")

if __name__ == "__main__":
    main()
