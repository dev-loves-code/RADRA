#!/usr/bin/env python3
"""Ablation table of the paper (Table 4, 2D timing and 3D rows) from results_proof/<variant>/*.txt.
Usage: ablation_table.py [results_proof dir]"""
import os, re, sys, glob
ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results_proof")
KINDS = ["adv", "sub", "hb", "map", "mapdel", "dv", "state", "split", "data", "ack"]
def parse(f):
    txt = open(f).read(); r2 = next((l for l in txt.splitlines() if l.startswith("RESULT2")), "")
    d = dict(re.findall(r"(\w+)=([^\s]+)", r2))
    rc = re.search(r"reachCovMin=([0-9.]+)", txt)
    return int(d.get("deliverable", 0)), int(d.get("delivered", 0)), sum(int(d.get(k + "Msgs", 0)) for k in KINDS), (float(rc[1]) if rc and "geocast" in f else None)
for title, pre in (("2D timing", "R__"), ("3D", "D__")):
    print("##", title)
    for v in ["fin", "fin_be", "abl_notorn", "abl_nosalv", "abl_dv"]:
        fs = sorted(glob.glob(f"{ROOT}/{v}/{pre}*.txt"))
        if not fs: continue
        rows = [parse(f) for f in fs]; dl = sum(r[0] for r in rows); lost = sum(r[0] - r[1] for r in rows)
        geo = [r[3] for r in rows if r[3] is not None]
        print(f"  {v:11s} runs {len(fs)} deliverable {dl} lost {lost} pdr {100*(dl-lost)/max(dl,1):.3f} geoWorst {min(geo) if geo else float('nan'):.3f} msgs/run {sum(r[2] for r in rows)//len(fs)}")
