#!/usr/bin/env python3
"""fate.py config.xml events.csv packets.csv : last events of the packets that were never delivered (deliverable or not)."""
import csv, sys, collections, re
X = open(sys.argv[1]).read()
reg = {i + 1: int(r) for i, r in enumerate(re.findall(r'<block position="[^"]*" region="(\d+)"', X))}
ev = [l.strip().split(',') for l in open(sys.argv[2])][1:]
dead = {int(e[2]) for e in ev if e[1] == 'kill'}
P = {int(p['pid']): p for p in csv.DictReader(open(sys.argv[3]))}
got = {int(e[3]) for e in ev if e[1] == 'deliver'}
by = collections.defaultdict(list)
for e in ev:
    if e[1] not in ('ctl', 'ev') and e[3].lstrip('-').isdigit(): by[int(e[3])].append(e)
fate = collections.Counter()
for pid, p in P.items():
    if pid in got: continue
    es = by.get(pid, [])
    k = tuple(e[1] for e in es if e[1] not in ('visit',))[-2:]
    last = int(es[-1][2]) if es else -1
    fate[(k, 'last module dead' if last in dead else 'alive', 'src dead' if int(p['src']) in dead else '', 'in dst region' if last > 0 and reg[last] == int(p['dstRegion']) else 'outside')] += 1
for k, n in fate.most_common(12): print(n, k)
