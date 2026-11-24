#!/usr/bin/env python3
"""Headline numbers of the proof campaign: lost, messages, bytes, latency, duplicates per group for fin, fin_be, final (old best effort), e2e_opt2 (old recommendation)."""
import os, re, glob
from collections import defaultdict
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results_proof")
F = lambda d, k: float(d.get(k, 0) or 0)
def load(v):
    return {os.path.basename(f)[:-4]: dict(re.findall(r"(\w+)=([^\s]+)", open(f).read())) for f in glob.glob(f"{ROOT}/{v}/*.txt")}
V = {v: load(v) for v in ["fin", "fin_be", "final", "e2e_opt2"]}
G = [("2D timing", lambda n: n.startswith("R__")), ("3D", lambda n: n.startswith("D__")), ("adversarial, no loss", lambda n: n.startswith("A__") and "loss" not in n),
     ("1 % link loss", lambda n: "loss1" in n), ("5 % link loss", lambda n: "loss5" in n), ("ring", lambda n: n.startswith("W__"))]
for lab, flt in G:
    print("##", lab)
    for v, R in V.items():
        s = defaultdict(float)
        for n, d in R.items():
            if not flt(n) or "delivered" not in d: continue
            s["runs"] += 1; s["dl"] += F(d, "deliverable"); s["dv"] += F(d, "delivered"); s["geo"] = min(s.get("geo", 1), F(d, "reachCovMin") if d.get("traffic") == "geocast" else 1)
            s["msgs"] += F(d, "dataMsgs") + F(d, "ackMsgs") + F(d, "ctlMsgs"); s["bytes"] += F(d, "dataBytes") + F(d, "ackBytes") + F(d, "ctlBytes")
            s["p95"] += F(d, "p95Latency_ms"); s["avg"] += F(d, "avgLatency_ms"); s["dup"] += F(d, "dupDeliveries")
        r = s["runs"]
        if r: print(f"  {v:9s} runs {r:4.0f} lost {s['dl']-s['dv']:6.0f} of {s['dl']:9.0f} geoWorst {s['geo']:.3f} msgs/run {s['msgs']/r:9.0f} KB/run {s['bytes']/r/1000:7.0f} avgLat {s['avg']/r:5.0f} p95 {s['p95']/r:5.0f} dup {s['dup']:6.0f}")
