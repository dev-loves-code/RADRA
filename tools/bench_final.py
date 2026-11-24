#!/usr/bin/env python3
"""Final campaign: RDRM v5 final (recovery buffer + suspect + split="walk") against v4 and the other options.
Writes results_final/*.txt. v4 runs and the lab baselines are reused from results/ when they already exist.
Usage: bench_final.py <VisibleSim>"""
import os, re, sys, shutil, subprocess, time
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, ".."))
VS = sys.argv[1]; TAG = sys.argv[2] if len(sys.argv) > 2 else "final"; SPLIT = sys.argv[3] if len(sys.argv) > 3 else "walk"
CFG, OLD, RES = f"{ROOT}/configs", f"{ROOT}/results_final", f"{ROOT}/results_{TAG}"
os.makedirs(RES, exist_ok=True)
FAILS = {"none": None, **{f"r5s{s}": f"random:5:{s}" for s in (1, 2, 3)}, **{f"r10s{s}": f"random:10:{s}" for s in (1, 2, 3)}}
SH = {"plain": (2, "plain"), "holes": (2, "holes"), "maze": (2, "maze"), "swiss": (2, "swiss:15:1"), "plain_k3": (3, "plain"), "plain_k4": (4, "plain"), "plain_k5": (5, "plain")}
V4 = ["profile=legacy", "salvage=off", "suspect=off"]
FINAL = ["salvage=on", "suspect=on", f"split={SPLIT}"]
BASE = ["cti=poison", "hbScope=all", "leader=elect", "balance=detour"]
jobs = []        # (name, k, kind, attrs)
def add(name, shape, attrs):
    k, kind = SH[shape]; jobs.append((name, k, kind, BASE + attrs))
for sh in ["plain", "holes", "maze", "swiss", "plain_k3", "plain_k4", "plain_k5"]:
    for tr in ["anycast", "unicast"]:
        for fl, fv in FAILS.items():
            f = [f"fail={fv}"] if fv else []
            t = [f"traffic={tr}"] + (["inreg=adapt"] if tr == "unicast" else [])
            add(f"A__{sh}__{tr}__{fl}__v4", sh, t + f + V4)
            add(f"A__{sh}__{tr}__{fl}__final", sh, t + f + FINAL)
for sh in ["plain", "holes", "maze", "swiss"]:                       # geocast after the failures
    for fl, fv in FAILS.items():
        if not fv: continue
        g = ["traffic=geocast", "geoEvery=1", "start=20000000", "failAt=15000000", f"fail={fv}"]
        add(f"G__{sh}__geocast__{fl}__v4", sh, g + V4); add(f"G__{sh}__geocast__{fl}__final", sh, g + FINAL)
for fl in ["torso", "armcut"]:
    for tr in ["anycast", "unicast"]:
        t = [f"traffic={tr}"] + (["inreg=adapt"] if tr == "unicast" else [])
        add(f"S__plain__{tr}__{fl}__v4", "plain", t + [f"fail={fl}"] + V4); add(f"S__plain__{tr}__{fl}__final", "plain", t + [f"fail={fl}"] + FINAL)
for sh in ["plain", "holes", "maze", "swiss", "plain_k3"]:            # the six in-region methods, final
    for m in ["flood", "dv", "mapbfs", "route", "geo", "adapt"]:
        for fl in ["none", "r5s1", "r10s1"]:
            f = [f"fail={FAILS[fl]}"] if FAILS[fl] else []
            add(f"B__{sh}__unicast-{m}__{fl}__final", sh, ["traffic=unicast", f"inreg={m}"] + f + FINAL)
for sh in ["plain", "plain_k3"]:                                    # load
    for bal in ["none", "detour"]:
        add(f"L__{sh}__hotspot__none__final__{bal}", sh, [a for a in BASE if not a.startswith("balance")] + [f"balance={bal}", "traffic=hotspot", "hotSrc=3456", "hotDst=1", "hotCount=10", "hotPeriod=400000"] + FINAL)
for sh, fl in [("plain", "r5s1"), ("plain_k3", "r5s2"), ("maze", "r10s1")]:   # stress: hotspot + failures
    add(f"T__{sh}__hotspot__{fl}__final", sh, ["traffic=hotspot", "hotSrc=3456", "hotDst=1", "hotCount=10", "hotPeriod=400000", f"fail={FAILS[fl]}"] + FINAL)
    add(f"T__{sh}__hotspot__{fl}__v4", sh, ["traffic=hotspot", "hotSrc=3456", "hotDst=1", "hotCount=10", "hotPeriod=400000", f"fail={FAILS[fl]}"] + V4)
for sh in ["plain", "plain_k3"]:                                    # lossy links (after start-up)
    for tr in ["anycast", "unicast"]:
        for loss in ["0.001", "0.01"]:
            t = [f"traffic={tr}"] + (["inreg=adapt"] if tr == "unicast" else [])
            add(f"W__{sh}__{tr}__loss{loss}__v4", sh, t + [f"linkLoss={loss}", "lossFrom=5000000"] + V4)
            add(f"W__{sh}__{tr}__loss{loss}__final", sh, t + [f"linkLoss={loss}", "lossFrom=5000000"] + FINAL)

def run(job):
    name, k, kind, a = job
    res = f"{RES}/{name}.txt"
    if os.path.exists(res) and "RESULT2" in open(res).read(): return name, 0
    old = f"{OLD}/{name}.txt"
    if name.endswith("__v4") and os.path.exists(old) and "RESULT2" in open(old).read():   # v4 did not change: reuse
        shutil.copy(old, res); return name, 0
    cfg = f"{CFG}/{TAG}_{name}.xml"
    subprocess.run(["python3", f"{HERE}/make_rdrm2.py", cfg, name, str(k), kind] + a, check=True, capture_output=True)
    t0 = time.time()
    p = subprocess.run([f"{VS}/applicationsBin/bbRDRM5/bbRDRM5", "-c", cfg, "-t", "-a", "1"], capture_output=True, text=True, timeout=7200, cwd=RES)
    out = re.sub(r"\x1b\[[0-9;]*m", "", p.stdout)
    open(res, "w").write("\n".join(l for l in out.splitlines() if l.startswith(("RESULT", "GEOCAST", "REACHCOV"))) + f"\nWALL {time.time() - t0:.1f}\n")
    for f in os.listdir(RES):
        if f.endswith(".csv"):
            try: os.remove(os.path.join(RES, f))
            except FileNotFoundError: pass
    return name, time.time() - t0

if __name__ == "__main__":
    jobs.sort(key=lambda j: -j[1])          # big robots first
    print(len(jobs), "runs", flush=True)
    t0 = time.time()
    with ThreadPoolExecutor(2) as ex:
        for i, (n, dt) in enumerate(ex.map(run, jobs)):
            if dt: print(f"[{i + 1}/{len(jobs)}] {n} {dt:.1f}s", flush=True)
    print(f"done in {(time.time() - t0) / 60:.1f} min")
