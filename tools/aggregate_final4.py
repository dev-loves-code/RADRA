#!/usr/bin/env python3
"""results_final + results_split + results (baselines) -> final_tables.json, final_summary.csv"""
import os, re, glob, csv, json, collections
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
def load(d):
    out = {}
    for f in glob.glob(f"{ROOT}/{d}/*.txt"):
        t = open(f).read()
        if "RESULT" not in t: continue
        v = dict(re.findall(r"(\w+)=([^\s]+)", t)); out[os.path.basename(f)[:-4]] = v
    return out
FIN, SPL, OLD = load("results_final4"), load("results_split"), load("results")
F = lambda x: float(x) if x not in ("", None) else 0.0
FAILS = ["r5s1", "r5s2", "r5s3", "r10s1", "r10s2", "r10s3"]
SH = ["plain", "holes", "maze", "swiss", "plain_k3", "plain_k4", "plain_k5"]
def pdr(vs):
    d = sum(F(v["delivered"]) for v in vs); dl = sum(F(v["deliverable"]) for v in vs); return round(100 * d / dl, 3) if dl else None
def miss(vs): return int(sum(F(v["deliverable"]) - F(v["delivered"]) for v in vs))
def get(g, sh, tr, fl, var, src=FIN): return src.get(f"{g}__{sh}__{tr}__{fl}__{var}")
T = {}
# delivery under failures, per shape
for tr in ["anycast", "unicast"]:
    T[f"rel_{tr}"] = {}
    for sh in SH:
        row = {}
        for var in ["v4", "final"]:
            vs = [x for x in (get("A", sh, tr, fl, var) for fl in FAILS) if x]
            row[var] = {"pdr": pdr(vs), "miss": miss(vs), "runs": len(vs), "perfect": sum(1 for v in vs if F(v["delivered"]) >= F(v["deliverable"]))}
        T[f"rel_{tr}"][sh] = row
allv = lambda var: [x for sh in SH for tr in ["anycast", "unicast"] for fl in FAILS for x in [get("A", sh, tr, fl, var)] if x]
T["overall"] = {var: {"pdr": pdr(allv(var)), "miss": miss(allv(var)), "runs": len(allv(var)), "perfect": sum(1 for v in allv(var) if F(v["delivered"]) >= F(v["deliverable"])),
                      "deliverable": int(sum(F(v["deliverable"]) for v in allv(var)))} for var in ["v4", "final"]}
# no failures
T["nofail"] = {sh: {tr: {var: {"pdr": pdr([get("A", sh, tr, "none", var)]), "data": F(get("A", sh, tr, "none", var)["dataMsgs"]), "ctl": F(get("A", sh, tr, "none", var)["ctlMsgs"])}
                         for var in ["v4", "final"]} for tr in ["anycast", "unicast"]} for sh in SH if get("A", sh, "anycast", "none", "final")}
# cost under failures
cost = {}
for sh in SH:
    a = [x for tr in ["anycast", "unicast"] for fl in FAILS for x in [get("A", sh, tr, fl, "v4")] if x]
    b = [x for tr in ["anycast", "unicast"] for fl in FAILS for x in [get("A", sh, tr, fl, "final")] if x]
    if not a or not b: continue
    cost[sh] = {"extraDataPct": round(100 * (sum(F(v["dataMsgs"]) for v in b) / sum(F(v["dataMsgs"]) for v in a) - 1), 1),
                "ctlPct": round(100 * (sum(F(v["ctlMsgs"]) for v in b) / sum(F(v["ctlMsgs"]) for v in a) - 1), 2),
                "splitMsgs": int(sum(F(v.get("splitMsgs")) for v in b)), "splitPctCtl": round(100 * sum(F(v.get("splitMsgs")) for v in b) / sum(F(v["ctlMsgs"]) for v in b), 2),
                "bufAvg": round(sum(F(v.get("avgPeakBufB")) for v in b) / len(b)), "bufMax": max(F(v.get("maxPeakBufB")) for v in b),
                "latV4": round(sum(F(v["avgLatency_ms"]) for v in a) / len(a), 1), "latFinal": round(sum(F(v["avgLatency_ms"]) for v in b) / len(b), 1)}
T["cost"] = cost
# scaling (no failures, final)
T["scale"] = {}
for sh in ["plain", "plain_k3", "plain_k4", "plain_k5"]:
    for tr in ["anycast", "unicast"]:
        v = get("A", sh, tr, "none", "final")
        if not v: continue
        n = F(v["N"]); st = sum(F(v.get(k + "Msgs")) for k in ["adv", "sub", "map", "dv"])
        T["scale"].setdefault(sh, {})[tr] = {"N": int(n), "startupPerModule": round(st / n, 1), "memB": F(v["avgMemB"]), "maxMemB": F(v["maxMemB"]), "lat": F(v["avgLatency_ms"]), "pdr": pdr([v]),
                                             "pdrFail": pdr([x for fl in FAILS for x in [get("A", sh, tr, fl, "final")] if x])}
# torn regions: modes compared on the same runs (A, shapes with split data), + geocast
torn = sorted({k.rsplit("__", 1)[0] for k, v in SPL.items() if k.endswith("split-grad") and F(v.get("splitMsgs")) > 0})
tornA = [b for b in torn if b.startswith("A__")]; tornG = [b for b in torn if b.startswith("G3__")]
def modeRuns(mode, bases):
    out = []
    for b in bases:
        if mode == "walk":
            g, sh, tr, fl, _ = b.split("__")
            x = FIN.get(f"{'A' if g == 'A' else 'G'}__{sh}__{tr}__{fl}__final")
        else: x = SPL.get(f"{b}__split-{mode}")
        if x: out.append(x)
    return out
T["torn"] = {"runsA": len(tornA), "runsG": len(tornG)}
for mode in ["none", "grad", "local", "seek", "walk"]:
    A = modeRuns(mode, tornA); G = modeRuns(mode, tornG)
    T["torn"][mode] = {"n": len(A), "pdr": pdr(A), "miss": miss(A), "perfect": sum(1 for v in A if F(v["delivered"]) >= F(v["deliverable"])),
                       "splitPerRun": round(sum(F(v.get("splitMsgs")) for v in A) / max(1, len(A))), "splitTotal": int(sum(F(v.get("splitMsgs")) for v in A)),
                       "splitPctCtl": round(100 * sum(F(v.get("splitMsgs")) for v in A) / max(1, sum(F(v["ctlMsgs"]) for v in A)), 2),
                       "memAvg": round(sum(F(v["avgMemB"]) for v in A) / max(1, len(A)), 1),
                       "geoN": len(G), "geoCov": round(100 * sum(F(v.get("reachCov")) for v in G) / max(1, len(G)), 2), "geoCovMin": round(100 * min([F(v.get("reachCovMin")) for v in G] or [1]), 1),
                       "perShape": {sh: int(sum(F(v.get("splitMsgs")) for b, v in zip(tornA, A) if b.split("__")[1] == sh)) for sh in ["plain", "holes", "maze", "swiss", "plain_k3", "plain_k4"]}}
# geocast (final campaign, after the failures): v4 vs final
T["geo"] = {sh: {var: {"cov": round(100 * sum(F(get("G", sh, "geocast", fl, var).get("reachCov")) for fl in FAILS) / 6, 2),
                       "min": round(100 * min(F(get("G", sh, "geocast", fl, var).get("reachCovMin")) for fl in FAILS), 1)} for var in ["v4", "final"]}
            for sh in ["plain", "holes", "maze", "swiss"] if all(get("G", sh, "geocast", fl, "final") and get("G", sh, "geocast", fl, "v4") for fl in FAILS)}
# structured
T["struct"] = {f"{fl}/{tr}": {var: pdr([get("S", "plain", tr, fl, var)]) for var in ["v4", "final"]} for fl in ["torso", "armcut"] for tr in ["anycast", "unicast"] if get("S", "plain", tr, fl, "final")}
# methods
meth = {}
for m in ["flood", "dv", "mapbfs", "route", "geo", "adapt"]:
    for fl in ["none", "r5s1", "r10s1"]:
        vs = [x for sh in ["plain", "holes", "maze", "swiss", "plain_k3"] for x in [FIN.get(f"B__{sh}__unicast-{m}__{fl}__final")] if x]
        if not vs: continue
        P = sum(F(v["packets"]) for v in vs); N = sum(F(v["N"]) for v in vs)
        meth.setdefault(m, {})[fl] = {"pdr": pdr(vs), "miss": miss(vs), "dataPerPkt": round(sum(F(v["dataMsgs"]) for v in vs) / P, 1),
            "startup": round(sum(sum(F(v.get(k + "Msgs")) - F(v.get(k + "AfterFail")) for k in ["adv", "sub", "map", "dv"]) for v in vs) / N, 1),
            "inregB": round(sum(F(v["avgInregB"]) for v in vs) / len(vs)), "maxInregB": max(F(v["maxInregB"]) for v in vs),
            "work": round(sum(F(v["workPerPacket"]) for v in vs) / len(vs)), "repair": round(sum(sum(F(v.get(k + "AfterFail")) for k in ["adv", "sub", "map", "mapdel", "dv", "split"]) for v in vs) / N, 1),
            "lat": round(sum(F(v["avgLatency_ms"]) for v in vs) / len(vs), 1)}
T["methods"] = meth
# load and stress
T["load"] = {k: {"maxLinkU": F(v["maxLinkU"]), "jain": F(v["jainTx"]), "p95": F(v["p95Latency_ms"]), "avg": F(v["avgLatency_ms"]), "maxTx": F(v["maxDataTx"]), "pdr": pdr([v])}
             for k, v in FIN.items() if k.startswith("L__")}
T["stress"] = {k: pdr([v]) for k, v in FIN.items() if k.startswith("T__")}
T["loss"] = {k: {"pdr": pdr([v]), "lostData": F(v.get("linkLostData")), "lostCtl": F(v.get("linkLostCtl")), "falsePos": F(v.get("falsePos"))} for k, v in FIN.items() if k.startswith("W__")}
# baselines (unchanged runs) next to final anycast
bl = {}
for sh in ["plain", "holes", "maze", "swiss", "plain_k3"]:
    for fl in ["none", "r5s1", "r10s1"]:
        r = get("A", sh, "anycast", fl, "final"); row = {}
        if r: row["rdrm"] = {"pdr": pdr([r]), "dataPerPkt": round(F(r["dataMsgs"]) / F(r["packets"]), 1), "allPerPkt": round((F(r["dataMsgs"]) + F(r["ctlMsgs"])) / F(r["packets"]), 1)}
        for b in ["bfs", "flooding"]:
            x = OLD.get(f"X__{sh}__anycast__{fl}__{b}")
            if x: row[b] = {"pdr": min(100.0, pdr([x])), "dataPerPkt": F(x["dataMsgsPerPacket"]), "allPerPkt": round(F(x["totalMsgs"]) / F(x["packets"]), 1)}
        bl[f"{sh}/{fl}"] = row
T["baselines"] = bl
json.dump(T, open(f"{ROOT}/final4_tables.json", "w"), indent=1)
print(json.dumps({k: T[k] for k in ["overall"]}, indent=1)); print(json.dumps(T["torn"], indent=1))
