#!/usr/bin/env python3
"""Tables for results_proof/: per campaign group and variant, delivered/deliverable, runs with losses, messages, buffers.
Usage: aggregate_proof.py [results_proof]  -> prints tables, writes proof_tables.json"""
import os, re, sys, glob, json
from collections import defaultdict
ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results_proof")
F = lambda v, k: float(v.get(k, 0) or 0)

def load(v):
    out = {}
    for f in glob.glob(f"{ROOT}/{v}/*.txt"):
        t = open(f).read()
        if "RESULT2" not in t: continue
        d = dict(re.findall(r"(\w+)=([^\s]+)", t)); out[os.path.basename(f)[:-4]] = d
    return out

def group(name):
    p = name.split("__")
    if p[0] == "R": return f"2D timing · {p[2]}"
    if p[0] == "A": return f"2D adversarial · {p[2]}"
    if p[0] == "W": return "ring (torn region, long detour)"
    if p[0] == "D": return f"3D · {p[2]} · {'wall' if p[-1] == 'wall' else 'random'}"
    return p[0]

def summarise(runs):
    s = defaultdict(float); s["runs"] = len(runs); bad = 0; worstCov = 1.0
    for n, v in runs.items():
        if v.get("traffic") == "geocast":
            c = F(v, "reachCovMin") if "reachCovMin" in v else 1.0
            worstCov = min(worstCov, c); s["geo"] += 1
            if c < 1: bad += 1
        else:
            dl, dv = F(v, "deliverable"), F(v, "delivered"); s["deliverable"] += dl; s["delivered"] += dv
            if dv < dl: bad += 1
        for k in ["dataMsgs", "ackMsgs", "ctlMsgs", "splitMsgs", "hbMsgs", "e2eResent", "resentDeliverable", "resentUndeliverable", "widens", "avgPeakBufB",
                  "dataBytes", "ackBytes", "ctlBytes", "dupDeliveries", "linkRetx", "unreachStop", "p95Latency_ms", "avgLatency_ms"]:
            s[k] += F(v, k)
        s["maxPeakBufB"] = max(s["maxPeakBufB"], F(v, "maxPeakBufB")); s["maxLatency_ms"] = max(s["maxLatency_ms"], F(v, "maxLatency_ms"))
        s["N"] += F(v, "N")
    s["badRuns"] = bad; s["worstCov"] = worstCov; s["lost"] = s["deliverable"] - s["delivered"]
    return dict(s)

if __name__ == "__main__":
    V = {v: load(v) for v in sorted(os.listdir(ROOT)) if os.path.isdir(f"{ROOT}/{v}")}
    groups = sorted({group(n) for v in V.values() for n in v})
    T = {}
    for g in groups:
        T[g] = {}
        for v, R in V.items():
            sel = {n: d for n, d in R.items() if group(n) == g}
            if sel: T[g][v] = summarise(sel)
    for g in groups:
        print(f"\n## {g}")
        for v, s in T[g].items():
            n = s["runs"]
            pdr = 100 * s["delivered"] / s["deliverable"] if s["deliverable"] else float("nan")
            print(f"  {v:10s} runs={n:4.0f} delivered={s['delivered']:9.0f}/{s['deliverable']:9.0f} ({pdr:8.4f} %) lost={s['lost']:5.0f} badRuns={s['badRuns']:3.0f}"
                  f" geoWorst={s['worstCov']:.3f} data/run={s['dataMsgs']/n:8.0f} ack/run={s['ackMsgs']/n:8.0f} ctl/run={s['ctlMsgs']/n:9.0f}"
                  f" split/run={s['splitMsgs']/n:6.0f} resent/run={s['e2eResent']/n:6.1f} (deliv {s['resentDeliverable']/n:5.1f}) widens/run={s['widens']/n:5.1f}"
                  f" dup={s['dupDeliveries']:5.0f} buf avg={s['avgPeakBufB']/n:6.0f} max={s['maxPeakBufB']:6.0f} p95lat={s['p95Latency_ms']/n:7.0f} maxlat={s['maxLatency_ms']:7.0f}")
    json.dump(T, open(os.path.join(ROOT, "..", "proof_tables.json"), "w"), indent=1)
