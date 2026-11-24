#!/usr/bin/env python3
"""The two lab baselines (bbFlooding, bbBFS) on the same robots, failures and anycast traffic as group A.
They read the same XML; we only add spacing="20000" (the RDRM default: one packet every 20 ms, robot-wide).
Note: in the baselines a failure is announced instantly by the simulator (no heartbeats), which favours them.
Usage: bench_baselines.py [VisibleSim dir]"""
import os, re, sys, subprocess, time
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, ".."))
VS = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/VisibleSim")
CFG, RES = f"{ROOT}/configs", f"{ROOT}/results"
jobs = []
for algo in ["bbBFS", "bbFlooding"]:
    for sh in ["plain", "holes", "maze", "swiss", "plain_k3"]:
        for fl in ["none", "r5s1", "r10s1"]:
            if algo == "bbFlooding" and sh == "plain_k3": continue        # ~10 M messages per run: skipped
            jobs.append((algo, sh, fl))

def run(j):
    algo, sh, fl = j
    name = f"X__{sh}__anycast__{fl}__{algo[2:].lower()}"
    res = f"{RES}/{name}.txt"
    if os.path.exists(res) and "RESULT" in open(res).read(): return name, 0
    src = open(f"{CFG}/A__{sh}__anycast__{fl}__v4.xml").read()
    src = re.sub(r'name="[^"]*"', f'name="{name}" spacing="20000"', src, count=1)
    cfg = f"{CFG}/{name}.xml"; open(cfg, "w").write(src)
    t0 = time.time()
    p = subprocess.run([f"{VS}/applicationsBin/{algo}/{algo}", "-c", cfg, "-t", "-a", "1"], capture_output=True, text=True, timeout=7200, cwd=RES)
    out = re.sub(r"\x1b\[[0-9;]*m", "", p.stdout)
    open(res, "w").write("\n".join(l for l in out.splitlines() if l.startswith("RESULT")) + f"\nWALL {time.time() - t0:.1f}\n")
    for f in os.listdir(RES):
        if f.endswith(".csv") and name in f: os.remove(os.path.join(RES, f))
    return name, time.time() - t0

if __name__ == "__main__":
    with ThreadPoolExecutor(2) as ex:
        for n, dt in ex.map(run, jobs): print(n, f"{dt:.1f}s", flush=True)
