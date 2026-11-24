#!/usr/bin/env python3
"""Ring robot: a closed loop 2 modules thick, 6 regions one after another. fail=cut kills region 1's middle cross-section,
so its two pieces meet only the long way round (far beyond local's reach). Usage: make_ring.py out.xml name L [key=value ...]"""
import sys, random
out, name, L = sys.argv[1], sys.argv[2], int(sys.argv[3]); kv = dict(a.split("=", 1) for a in sys.argv[4:])
W, H = L, 12                                   # loop around a W x H rectangle
loop = [(x, 0) for x in range(W)] + [(W - 1, z) for z in range(1, H)] + [(x, H - 1) for x in range(W - 2, -1, -1)] + [(0, z) for z in range(H - 2, 0, -1)]
n = len(loop); cells = {}
for i, (x, z) in enumerate(loop):
    r = 1 + (i * 6) // n
    for t in range(2):                         # 2 thick (inwards)
        p = (x + (t if x == 0 else -t if x == W - 1 else 0), z + (t if z == 0 else -t if z == H - 1 else 0))
        cells.setdefault(p, r)
start = int(kv.pop("start", 5000000)); failAt = int(kv.pop("failAt", 15000000)); fail = kv.pop("fail", "cut")
mid = loop[(n // 12)]                          # middle of region 1
F = {p for p in cells if p[0] == mid[0] and cells[p] == 1} if fail == "cut" else set()
exp = {"name": name, "start": start, "dmax": n + 10}; exp.update(kv)
if F: exp["failAt"] = failAt
rt = {}
for r in range(1, 7):
    pts = sorted(p for p in cells if cells[p] == r); rt[r] = pts[len(pts) // 2]
Lx = ['<?xml version="1.0" standalone="no" ?>', '<vs>', f'  <experiment {" ".join(f"{a}=\"{v}\"" for a, v in exp.items())}/>', f'  <world gridSize="{W},1,{H}">', '    <blockList color="200,200,200">']
for p in sorted(cells, key=lambda p: (p[1], p[0])):
    e = (' root="true"' if rt[cells[p]] == p else '') + (' fail="true"' if p in F else '')
    Lx.append(f'      <block position="{p[0]},0,{p[1]}" region="{cells[p]}"{e}/>')
Lx += ['    </blockList>', '  </world>', '</vs>']
open(out, "w").write("\n".join(Lx) + "\n"); print(out, "N=", len(cells), "failed", len(F))
