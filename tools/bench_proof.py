#!/usr/bin/env python3
"""Validation of the guaranteed version (local + widening + end-to-end acknowledgements) against the final v5.
Usage: bench_proof.py <VisibleSim> [part ...]   parts: timing adv ring 3d copylog (default: all)
Writes results_proof/<variant>/<name>.txt"""
import os, re, sys, subprocess, time, glob
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, ".."))
VS = sys.argv[1]; PARTS = sys.argv[2:] or ["timing", "adv", "ring", "3d", "copylog"]
ONLY = os.environ.get("VARIANTS")
OUT = f"{ROOT}/results_proof"; CFG = f"{ROOT}/configs_proof"
BASE = ["cti=poison", "hbScope=all", "leader=elect", "balance=detour", "salvage=on", "suspect=on", "split=local", "tail=30000000"]
OPT = ["hbAdapt=on", "ackCopy=off", "giveUp=off"]
VAR = {"fin": [], "fin_be": ["e2e=off"], "abl_dv": ["e2e=off", "salvage=off", "split=none", "widen=off", "gcAck=off"], "abl_notorn": ["e2e=off", "split=none", "widen=off", "gcAck=off"], "abl_nosalv": ["e2e=off", "salvage=off"], "final_opt": OPT, "e2e_opt": ["e2e=on"] + OPT, "final_opt2": OPT, "e2e_opt2": ["e2e=on"] + OPT, "final": [], "e2e": ["e2e=on"], "e2e_nolog": ["e2e=on", "copyLog=off"], "nowiden": ["widen=off"]}
TIMING = {
    "mid": ["failAt=15000000"], "early": ["failAt=8000000"], "late": ["failAt=30000000"],
    "casc3": ["failAt=15000000", "failSpread=3000000"], "casc15": ["failAt=10000000", "failSpread=15000000"],
    "fast": ["failAt=15000000", "spacing=10000"], "casc1": ["failAt=15000000", "failSpread=1000000"],
    "waves": ["failAt=12000000", "failSpread=8000000"]}
SH2 = {"plain": (2, "plain"), "holes": (2, "holes"), "maze": (2, "maze"), "swiss": (2, "swiss:15:1"), "plain_k3": (3, "plain")}
jobs = []          # (variant, name, generator, k, shape, attrs)
def add(variants, name, gen, k, shape, attrs):
    if ONLY: variants = [x for x in ONLY.split(",")] if any(v in ("final", "e2e") for v in variants) else []
    for v in variants: jobs.append((v, name, gen, k, shape, BASE + VAR[v] + attrs))

if "timing" in PARTS:
    for sh, (k, kind) in SH2.items():
        for s in range(1, 11):
            for tm, ex in TIMING.items():
                add(["final", "e2e"], f"R__{sh}__unicast__s{s}__{tm}", "2d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail=random:10:{s}", f"spreadSeed={s}"] + ex)
            for tm in ["casc1", "casc3", "casc15"]:
                add(["final", "e2e"], f"R__{sh}__anycast__s{s}__{tm}", "2d", k, kind, ["traffic=anycast", f"fail=random:10:{s}", f"spreadSeed={s}"] + TIMING[tm])
        if sh != "plain_k3":
            for s in range(1, 11):
                add(["final", "e2e"], f"R__{sh}__geocast__s{s}__casc15", "2d", k, kind, ["traffic=geocast", "geoEvery=1", f"fail=random:10:{s}", f"spreadSeed={s}", "failAt=12000000", "failSpread=15000000", "start=14000000"])
if "nowidentiming" in PARTS:    # cost of widening: the unicast timing runs with widening off
    for sh, (k, kind) in SH2.items():
        for s in range(1, 11):
            for tm, ex in TIMING.items():
                add(["nowiden"], f"R__{sh}__unicast__s{s}__{tm}", "2d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail=random:10:{s}", f"spreadSeed={s}"] + ex)
if "copylog" in PARTS:          # is the per-hop copy log still useful once the source keeps the packet?
    for sh, (k, kind) in SH2.items():
        if sh == "plain_k3": continue
        for s in range(1, 11):
            for tm in ["casc3", "casc15", "mid"]:
                add(["e2e_nolog"], f"R__{sh}__unicast__s{s}__{tm}", "2d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail=random:10:{s}", f"spreadSeed={s}"] + TIMING[tm])
if "adv" in PARTS:
    BLOCK = "cells:" + ";".join(f"{x},{z}" for x in range(7, 11) for z in range(15, 19))   # a 4 x 4 block of the torso dies at once
    ADV = {"slowhb": ["hbPeriod=1000000", "failAt=15000000", "failSpread=1000000"],
           "heavy": ["spacing=4000", "failAt=15000000", "failSpread=1000000"],
           "loss1": ["linkLoss=0.01", "lossFrom=5000000", "failAt=15000000", "failSpread=3000000", "tail=90000000"],
           "loss5": ["linkLoss=0.05", "lossFrom=5000000", "failAt=15000000", "failSpread=3000000", "tail=90000000"]}
    for sh in ["plain", "holes", "swiss"]:
        k, kind = SH2[sh]
        for s in range(1, 6):
            for a, ex in ADV.items():
                add(["final", "e2e"], f"A__{sh}__{a}__s{s}", "2d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail=random:10:{s}", f"spreadSeed={s}"] + ex)
        add(["final", "e2e"], f"A__{sh}__block", "2d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail={BLOCK}", "failAt=15000000"])
if "ring" in PARTS:
    for L in [40, 60, 90]:
        add(["nowiden", "final", "e2e"], f"W__ring{L}", "ring", L, "-", ["traffic=unicast", "inreg=adapt"])
if "3d" in PARTS:
    SH3 = {"h3": (2, "h3"), "tunnels": (2, "tunnels"), "cavity": (2, "cavity"), "maze3": (2, "maze3"), "swiss3": (2, "swiss3:15:1"), "cube": (3, "cube")}
    for sh, (k, kind) in SH3.items():
        for s in range(1, 6):
            for tm in ["mid", "casc3", "casc15", "casc1"]:
                add(["final", "e2e"], f"D__{sh}__unicast__s{s}__{tm}", "3d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail=random:10:{s}", f"spreadSeed={s}"] + TIMING[tm])
            add(["final", "e2e"], f"D__{sh}__anycast__s{s}__casc3", "3d", k, kind, ["traffic=anycast", f"fail=random:10:{s}", f"spreadSeed={s}"] + TIMING["casc3"])
            if sh != "cube":
                add(["final", "e2e"], f"D__{sh}__unicast__s{s}__wall", "3d", k, kind, ["traffic=unicast", "inreg=adapt", f"fail=wall+random:3:{s}", f"spreadSeed={s}", "failAt=15000000", "failSpread=3000000"])
                if s <= 3: add(["final", "e2e"], f"D__{sh}__geocast__s{s}__wall", "3d", k, kind, ["traffic=geocast", "geoEvery=1", f"fail=wall+random:3:{s}", f"spreadSeed={s}", "failAt=12000000", "failSpread=3000000", "start=14000000"])

GEN = {"2d": "make_rdrm2.py", "3d": "make_rdrm3d.py", "ring": "make_ring.py"}
def run(j):
    v, name, gen, k, shape, attrs = j
    res_dir = f"{OUT}/{v}"; os.makedirs(res_dir, exist_ok=True); os.makedirs(f"{CFG}/{v}", exist_ok=True)
    res = f"{res_dir}/{name}.txt"
    if os.path.exists(res) and "RESULT2" in open(res).read(): return 0
    cfg = f"{CFG}/{v}/{name}.xml"
    # later attributes win (tail=90 s for the lossy runs)
    d = {}
    for a in attrs: kk, vv = a.split("=", 1); d[kk] = vv
    args = [f"{kk}={vv}" for kk, vv in d.items()]
    if gen == "ring": cmd = ["python3", f"{HERE}/{GEN[gen]}", cfg, name, str(k)] + args
    else: cmd = ["python3", f"{HERE}/{GEN[gen]}", cfg, name, str(k), shape] + args
    subprocess.run(cmd, check=True, capture_output=True)
    t0 = time.time(); wd = f"/tmp/proofrun_{os.getpid()}_{abs(hash(name)) % 100000}"; os.makedirs(wd, exist_ok=True)
    p = subprocess.run([f"{VS}/applicationsBin/bbRDRM5/bbRDRM5", "-c", cfg, "-t", "-a", "1"], capture_output=True, text=True, timeout=5400, cwd=wd)
    out = re.sub(r"\x1b\[[0-9;]*m", "", p.stdout)
    open(res, "w").write("\n".join(l for l in out.splitlines() if l.startswith(("RESULT", "GEOCAST", "REACHCOV", "E2E", "WARNING"))) + f"\nWALL {time.time() - t0:.1f}\n")
    for f in glob.glob(f"{wd}/*"):
        try: os.remove(f)
        except OSError: pass
    try: os.rmdir(wd)
    except OSError: pass
    return time.time() - t0

if __name__ == "__main__":
    jobs.sort(key=lambda j: (j[2] != "3d", -j[3]))     # long 3D runs first
    print(len(jobs), "runs", flush=True)
    done = 0
    with ThreadPoolExecutor(2) as ex:
        for dt in ex.map(run, jobs):
            done += 1
            if done % 50 == 0: print(done, "done", flush=True)
    print("finished", flush=True)
