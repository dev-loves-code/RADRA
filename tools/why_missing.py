#!/usr/bin/env python3
"""Explain the missing packets of one run: why_missing.py config.xml events.csv packets.csv
(events = RR_LOG output, packets = results_rdrm5_<name>.csv). For each deliverable packet that was not delivered,
says whether its target sits in a piece of its region that the rest of the region cannot reach any more."""
import re, sys, csv, collections
X = open(sys.argv[1]).read()
B = re.findall(r'<block position="(\d+),(\d+),(\d+)" region="(\d+)"', X)
pos = {i + 1: (int(x), int(y), int(z)) for i, (x, y, z, r) in enumerate(B)}; reg = {i + 1: int(r) for i, (x, y, z, r) in enumerate(B)}
ev = [l.strip().split(',') for l in open(sys.argv[2])][1:]
dead = {int(e[2]) for e in ev if e[1] == 'kill'}; at = {p: i for i, p in pos.items()}
alive = {i for i in pos if i not in dead}
def comps(nodes):
    nodes = set(nodes); out = {}
    k = 0
    while nodes:
        s = nodes.pop(); st = [s]; out[s] = k
        while st:
            u = st.pop(); x, y, z = pos[u]
            for d in [(1,0,0),(-1,0,0),(0,0,1),(0,0,-1),(0,1,0),(0,-1,0)]:
                v = at.get((x + d[0], y + d[1], z + d[2]))
                if v in nodes: nodes.remove(v); out[v] = k; st.append(v)
        k += 1
    return out
robot = comps(alive)
piece = {}; size = collections.Counter()
for r in set(reg.values()):
    c = comps([i for i in alive if reg[i] == r])
    for m, k in c.items(): piece[m] = (r, k); size[(r, k)] += 1
P = list(csv.DictReader(open(sys.argv[3])))
why = collections.Counter()
for p in P:
    s, t = int(p['src']), int(p['dst'])
    if p['hops'] not in ('', '-1'): continue                          # delivered
    if t in dead or s in dead or robot.get(s) != robot.get(t): continue   # not deliverable
    r = reg[t]; big = max((k for k in size if k[0] == r), key=lambda k: size[k])
    why['target in a cut-off piece of its region' if piece[t] != big else 'target in the main piece'] += 1
print(dict(why))
