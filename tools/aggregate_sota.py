#!/usr/bin/env python3
"""Tables for results_sota/: per method, traffic and failure class (none / random 5-10 % / rootfail), small robots (340-400) vs 900."""
import re, glob, os, json, sys
from collections import defaultdict
g = lambda d, k: float(d.get(k, 0) or 0)
S = defaultdict(lambda: defaultdict(float))
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
DIRS = sys.argv[1:] or ["results_sota"]          # e.g. results_sota2 results_sota (later dirs only add methods the first ones lack)
files, seenV = [], set()
for D in DIRS:
    fs = glob.glob(f"{R}/{D}/*.txt"); vs = {os.path.basename(f)[:-4].split("__")[-1] for f in fs}
    keep = lambda v: v not in seenV and (D == DIRS[0] or v in ("bfs", "flooding"))   # older dirs: only the unchanged lab baselines
    files += [f for f in fs if keep(os.path.basename(f)[:-4].split("__")[-1])]; seenV |= vs
for f in files:
    n = os.path.basename(f)[:-4]; _, sh, tr, fl, v = n.split("__")
    d = dict(re.findall(r"(\w+)=([^\s]+)", open(f).read()))
    cls = "none" if fl == "none" else ("rootfail" if fl == "rootfail" else "random")
    size = "900" if sh == "plain_k3" else "small"
    k = (tr, cls, size, v); s = S[k]
    pk = g(d, "packets"); dl = g(d, "deliverable") or pk; dv = min(g(d, "delivered"), dl)
    base = v in ("bfs", "flooding")
    allm = g(d, "totalMsgs") if base else g(d, "dataMsgs") + g(d, "ctlMsgs") + g(d, "ackMsgs")
    # GPSR needs only data + neighbour beacons (heartbeats): gradient/map messages are not counted for it
    if v == "gpsr": allm = g(d, "dataMsgs") + g(d, "hbMsgs")
    allb = g(d, "dataBytes") + g(d, "setupBytes") + g(d, "repairBytes") if base else g(d, "dataBytes") + g(d, "ctlBytes") + g(d, "ackBytes")
    if v == "gpsr": allb = g(d, "dataBytes") + g(d, "hbMsgs")
    s["runs"] += 1; s["pk"] += pk; s["dl"] += dl; s["dv"] += dv; s["data"] += g(d, "dataMsgs"); s["all"] += allm; s["bytes"] += allb
    s["hops"] += g(d, "avgHops"); s["stretch"] += g(d, "stretch") or g(d, "avgStretch"); s["lat"] += g(d, "avgLatency_ms"); s["p95"] += g(d, "p95Latency_ms")
    s["maxlat"] = max(s["maxlat"], g(d, "maxLatency_ms")); s["bad"] += dv < dl
out = {}
for k in sorted(S):
    s = S[k]; n = s["runs"]
    row = dict(runs=n, pdr=100 * s["dv"] / s["dl"] if s["dl"] else 0, lost=s["dl"] - s["dv"], badRuns=s["bad"], dataPerPkt=s["data"] / s["pk"], allPerPkt=s["all"] / s["pk"],
               bytesPerPkt=s["bytes"] / s["pk"], hops=s["hops"] / n, stretch=s["stretch"] / n, avgLat=s["lat"] / n, p95=s["p95"] / n, maxLat=s["maxlat"])
    out["|".join(k)] = row
    print("%-8s %-9s %-5s %-9s runs %2d  delivered %7.3f%% lost %5d bad %2d  data/pkt %6.1f all/pkt %6.1f bytes/pkt %6.0f hops %5.1f stretch %5.2f lat %5.0f p95 %5.0f" %
          (*k, n, row["pdr"], row["lost"], row["badRuns"], row["dataPerPkt"], row["allPerPkt"], row["bytesPerPkt"], row["hops"], row["stretch"], row["avgLat"], row["p95"]))
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", DIRS[0].replace("results_", "") + "_tables.json"), "w"), indent=1)
