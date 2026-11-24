#!/usr/bin/env python3
"""Comparison with the state of the art on identical configs (the group-A configs of the main campaign).
Methods: RDRM v4, v5 best effort, v5 optimized (adaptive heartbeats), v5 guaranteed (receipts),
Le & Ngo 2016 (one gradient per region, from its root only), GPSR (Karp & Kung 2000, robot-wide, grid version),
and the lab baselines bbBFS and bbFlooding (anycast only).
Failure settings: none, 5 % and 10 % random (3 seeds each), and rootfail (5 % random + the root of regions 1 and 2 die).
Usage: bench_sota.py <VisibleSim>   -> results_sota/"""
import os, re, sys, subprocess
from concurrent.futures import ThreadPoolExecutor
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); VS = sys.argv[1]
OUT = f"{ROOT}/results_sota2"; os.makedirs(OUT, exist_ok=True)
STRIP = ["salvage", "suspect", "split", "e2e", "hbAdapt", "ackCopy", "baseline", "profile"]
V = {"final": '', "final_be": 'e2e="off"',                       # the delivered algorithm (profile final) and its best-effort mode
     "lengo": 'profile="legacy" baseline="lengo"', "gpsr": 'profile="legacy" baseline="gpsr"'}
SHAPES = ["plain", "holes", "maze", "swiss", "plain_k3"]
FAILS = ["none", "r5s1", "r5s2", "r5s3", "r10s1", "r10s2", "r10s3", "rootfail"]
jobs = []
for sh in SHAPES:
    for tr in ["anycast", "unicast"]:
        for fl in FAILS:
            for v in V:
                if not (v == "lengo" and tr == "unicast"): jobs.append(("bbRDRM5", v, sh, tr, fl))   # Le & Ngo routes to a root: anycast only
            # bbBFS / bbFlooding are separate apps, unchanged: their results in results_sota are reused

def config(sh, tr, fl):
    src = open(f"{ROOT}/configs/A__{sh}__{tr}__{fl if fl != 'rootfail' else 'r5s1'}__v4.xml").read()
    if fl == "rootfail":                       # the roots of regions 1 and 2 die with the others (Le & Ngo's single point of failure)
        src = re.sub(r'(<block [^>]*region="[12]"[^>]*root="true")', r'\1 fail="true"', src)
        src = re.sub(r'(fail="true")( fail="true")+', r'\1', src)
    return src

def run(j):
    app, v, sh, tr, fl = j
    name = f"S__{sh}__{tr}__{fl}__{v}"; res = f"{OUT}/{name}.txt"
    if os.path.exists(res) and "RESULT" in open(res).read(): return
    src = config(sh, tr, fl)
    if app == "bbRDRM5":
        for a in STRIP: src = re.sub(rf' {a}="[^"]*"', "", src)
        src = re.sub(r'name="[^"]*"', f'name="{name}" {V[v]}', src, count=1)
    else:
        src = re.sub(r'name="[^"]*"', f'name="{name}" spacing="20000"', src, count=1)
    cfg = f"{OUT}/{name}.xml"; open(cfg, "w").write(src)
    try:
        p = subprocess.run([f"{VS}/applicationsBin/{app}/{app}", "-c", cfg, "-t", "-a", "1"], capture_output=True, text=True, timeout=5400, cwd="/tmp")
        out = re.sub(r"\x1b\[[0-9;]*m", "", p.stdout)
    except subprocess.TimeoutExpired: out = "RESULT timeout=1"
    open(res, "w").write("\n".join(l for l in out.splitlines() if l.startswith(("RESULT", "E2E"))) + "\n")

if __name__ == "__main__":
    jobs.sort(key=lambda j: (j[2] != "plain_k3", j[1] != "gpsr"))
    print(len(jobs), "runs", flush=True)
    with ThreadPoolExecutor(2) as ex: list(ex.map(run, jobs))
    print("finished", flush=True)
