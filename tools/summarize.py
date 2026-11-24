#!/usr/bin/env python3
"""results/*.txt -> summary.csv (one row per run) and grouped means printed to the terminal."""
import os, re, glob, csv, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
KEEP = ["N", "killed", "packets", "deliverable", "delivered", "drops", "lostAtDead", "dataMsgs", "ctlMsgs", "hbMsgs", "advMsgs", "mapMsgs", "dvMsgs",
        "advAfterFail", "mapAfterFail", "mapdelAfterFail", "dvAfterFail", "subAfterFail", "subMsgs", "stretch", "avgLatency_ms", "p95Latency_ms",
        "avgMemB", "maxMemB", "avgInregB", "maxInregB", "workPerPacket", "salvaged", "resentMsgs", "parked", "parkExpired", "logFull",
        "dupDiscards", "dupDeliveries", "suspectAvoid", "avgPeakBufB", "maxPeakBufB", "falsePos", "avgDetect_ms", "maxLinkU", "jainTx", "maxDataTx",
        "avgCoverage", "minCoverage", "coverageAfterDetection", "geocasts", "msgsPerGeocast", "detours",
        "totalMsgs", "dataMsgsPerPacket", "avgMemEntries", "maxMemEntries", "avgStretch", "setupMsgs", "repairMsgs", "pieceCuts", "pieceExits"]
rows = []
for f in sorted(glob.glob(f"{ROOT}/results/*.txt")):
    txt = open(f).read()
    if "RESULT" not in txt: continue
    v = dict(re.findall(r"(\w+)=([^\s]+)", txt))
    name = os.path.basename(f)[:-4]; p = name.split("__")
    r = dict(run=name, group=p[0], shape=p[1], traffic=p[2], fail=p[3], variant=p[4], tag=p[5] if len(p) > 5 else "")
    r.update({k: v.get(k, "") for k in KEEP})
    try:
        d, dl = float(r["delivered"]), float(r["deliverable"])
        r["pdr"] = round(100 * d / dl, 3) if dl else ""
        r["missing"] = int(dl - d)
        r["startup"] = sum(float(v.get(k, 0)) for k in ["advMsgs", "subMsgs", "mapMsgs", "dvMsgs"]) - sum(float(v.get(k + "AfterFail", 0)) for k in ["adv", "sub", "map", "dv"])
        r["repair"] = sum(float(v.get(k + "AfterFail", 0)) for k in ["adv", "sub", "map", "mapdel", "dv"])
        r["dataPerPkt"] = round(float(v["dataMsgs"]) / max(1, float(v["packets"])), 3)
    except Exception: pass
    rows.append(r)
cols = list(rows[0].keys()) if rows else []
for r in rows:
    for c in r:
        if c not in cols: cols.append(c)
with open(f"{ROOT}/summary.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, cols); w.writeheader(); w.writerows(rows)
print(f"{len(rows)} runs -> summary.csv")
