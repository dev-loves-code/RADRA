#!/usr/bin/env python3
"""RDRM v5 benchmark campaign: generates the configs, runs them (2 at a time), writes results/*.txt and summary.csv.

Usage: bench.py [VisibleSim dir] [groups]     groups: any of A G S B C L (default all)
Variants (same binary, bbRDRM5):
  v4      salvage=off suspect=off   (bit-for-bit identical to bbRDRM4, checked)
  rec     salvage=on                 (recovery buffer only)
  final   salvage=on suspect=on      (recovery buffer + suspect after 1 missed heartbeat)
"""
import os, re, sys, subprocess, csv, time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
VS = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/VisibleSim")
GROUPS = sys.argv[2] if len(sys.argv) > 2 else "AGSBCLP"
BIN = f"{VS}/applicationsBin/bbRDRM5/bbRDRM5"
ROOT = os.path.abspath(os.path.join(HERE, ".."))
CFG, RES = f"{ROOT}/configs", f"{ROOT}/results"
os.makedirs(CFG, exist_ok=True); os.makedirs(RES, exist_ok=True)

SHAPES = {"plain": (2, "plain"), "holes": (2, "holes"), "maze": (2, "maze"), "swiss": (2, "swiss:15:1"),
          "plain_k3": (3, "plain"), "plain_k4": (4, "plain"), "plain_k5": (5, "plain")}
VARIANTS = {"v4": dict(salvage="off", suspect="off"), "rec": dict(salvage="on", suspect="off"), "final": dict(salvage="on", suspect="on"), "piece": dict(salvage="on", suspect="on", piece="on")}
BASE = dict(cti="poison", hbScope="all", leader="elect", balance="detour")
FAILS = {"none": None, **{f"r5s{s}": f"random:5:{s}" for s in (1, 2, 3)}, **{f"r10s{s}": f"random:10:{s}" for s in (1, 2, 3)}}

jobs = []   # (name, k, shape, attrs)
def add(group, shape, traffic, fail, variant, **extra):
    k, kind = SHAPES[shape]
    name = f"{group}__{shape}__{traffic}__{fail}__{variant}" + (f"__{extra['_tag']}" if "_tag" in extra else "")
    a = dict(BASE); a.update(VARIANTS[variant])
    if traffic.startswith("unicast"):
        a["traffic"] = "unicast"; a["inreg"] = traffic.split("-")[1] if "-" in traffic else "adapt"
    else: a["traffic"] = traffic
    if FAILS.get(fail): a["fail"] = FAILS[fail]
    elif fail in ("torso", "armcut"): a["fail"] = fail
    a.update({k2: v for k2, v in extra.items() if not k2.startswith("_")})
    jobs.append((name, k, kind, a))

if "A" in GROUPS:   # reliability: v4 vs recovery vs recovery + suspect
    for sh in ["plain", "holes", "maze", "swiss", "plain_k3", "plain_k4"]:
        for tr in ["anycast", "unicast"]:
            for fl in FAILS:
                for v in ["v4", "rec", "final"]: add("A", sh, tr, fl, v)
if "G" in GROUPS:   # geocast reliability
    for sh in ["plain", "maze", "swiss"]:
        for fl in ["none", "r5s1", "r5s2", "r5s3"]:
            for v in ["v4", "final"]: add("G", sh, "geocast", fl, v)
if "S" in GROUPS:   # structured failures
    for fl in ["torso", "armcut"]:
        for tr in ["anycast", "unicast"]:
            for v in ["v4", "rec", "final"]: add("S", "plain", tr, fl, v)
if "B" in GROUPS:   # the six in-region unicast methods, final version
    for sh in ["plain", "holes", "maze", "swiss", "plain_k3"]:
        for m in ["flood", "dv", "mapbfs", "route", "geo", "adapt"]:
            for fl in ["none", "r5s1"]: add("B", sh, f"unicast-{m}", fl, "final")
if "C" in GROUPS:   # scaling (k2..k4 come from group A)
    for tr in ["anycast", "unicast"]: add("C", "plain_k5", tr, "none", "final")
if "P" in GROUPS:   # experimental: piece-aware gradient (a cut-off piece of a region stops posing as the region)
    for sh in ["plain", "holes", "swiss"]:
        for fl in [f for f in FAILS if f != "none"]: add("P", sh, "unicast", fl, "piece")
if "L" in GROUPS:   # load: hotspot, limbs -> head, balancer off / on
    for sh in ["plain", "plain_k3"]:
        for bal in ["none", "detour"]:
            add("L", sh, "hotspot", "none", "final", balance=bal, hotSrc="3456", hotDst="1", hotCount="10", hotPeriod="400000", _tag=bal)

def gen(name, k, kind, a):
    out = f"{CFG}/{name}.xml"
    if not os.path.exists(out):
        args = ["python3", f"{HERE}/make_rdrm2.py", out, name, str(k), kind] + [f"{x}={y}" for x, y in a.items()]
        subprocess.run(args, check=True, capture_output=True)
    return out

def run(job):
    name, k, kind, a = job
    res = f"{RES}/{name}.txt"
    if os.path.exists(res) and "RESULT2" in open(res).read(): return name, 0
    cfg = gen(name, k, kind, a)
    t0 = time.time()
    p = subprocess.run([BIN, "-c", cfg, "-t", "-a", "1"], capture_output=True, text=True, timeout=3600, cwd=RES)
    out = re.sub(r"\x1b\[[0-9;]*m", "", p.stdout)
    keep = [l for l in out.splitlines() if l.startswith(("RESULT", "GEOCAST", "REGIONGEO", "WARNING"))]
    open(res, "w").write("\n".join(keep) + f"\nWALL {time.time() - t0:.1f}\n")
    for f in os.listdir(RES):                                   # per-packet / per-module CSVs: not kept
        if f.endswith(".csv") and name in f: os.remove(os.path.join(RES, f))
    return name, time.time() - t0

if __name__ == "__main__":
    print(f"{len(jobs)} runs")
    t0 = time.time()
    with ThreadPoolExecutor(2) as ex:
        for i, (n, dt) in enumerate(ex.map(run, jobs)):
            if dt: print(f"[{i + 1}/{len(jobs)}] {n} {dt:.1f}s", flush=True)
    print(f"done in {(time.time() - t0) / 60:.1f} min")
