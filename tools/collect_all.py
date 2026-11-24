#!/usr/bin/env python3
"""Every run of every campaign on disk in one table, so nothing has to be rerun.
Writes all_runs.csv (one row per run) and campaigns.csv (what each campaign was).
Usage: collect_all.py [bench dir]"""
import os, re, sys, csv, glob
B = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

CAMPAIGNS = {   # folder: (code version, what it tested)
    "results":         ("v4 / v5-early", "Main campaign groups A (failures), B (in-region methods), G (geocast), L (load), P/S/T (structured, stress), X (lab baselines bbBFS, bbFlooding)"),
    "results_final":   ("v5 (walk)", "Main campaign, 368 runs, final version with split=walk (first full validation)"),
    "results_final2":  ("v5 (walk, fixes)", "Main campaign rerun after walk fixes"),
    "results_final3":  ("v5 (local)", "Main campaign rerun with split=local: 0 lost of 332,624"),
    "results_split":   ("v5 torn-region study", "Torn regions: none / grad / local / seek / walk compared on the same failures"),
    "results_timing":  ("v5 (walk, first)", "Timing campaign, 220 runs: when failures happen (mid, early, late, cascades, fast, waves)"),
    "results_timing2": ("v5 (walk)", "Timing campaign rerun"),
    "results_timing3": ("v5 (walk, first fixes)", "Timing campaign, 590 runs"),
    "results_twalk3":  ("v5 (walk)", "Timing campaign subset, walk"),
    "results_twalk4":  ("v5 (walk, all fixes + flood backup)", "Timing campaign, 590 runs: 1 lost, 36k torn-region msgs/run"),
    "results_tlocal":  ("v5 (local)", "Timing campaign, 590 runs, local, first"),
    "results_tlocal2": ("v5 (local, quiet parking)", "Timing campaign rerun (regression found: waiting-room overflow)"),
    "results_tlocal3": ("v5 (local)", "Timing campaign rerun"),
    "results_tlocal4": ("v5 (local, 4 s window)", "Timing campaign rerun"),
    "results_tlocal5": ("v5 (local, final best effort)", "Timing campaign, 590 runs: 0 lost of 929,890"),
    "results_proof_v1":("v5 guaranteed (widening always)", "Proof campaign, 1,815 runs: final vs e2e, adversarial, ring, 3D"),
    "results_proof/abl": ("ABLATION", "see results_proof/abl_dv, abl_notorn, abl_nosalv"),
    "results_preadvsync": ("FINAL before advSync", "First final runs (proof fin/fin_be, main final4, sota2 final/final_be) before the gradient re-sync fix; superseded"),
    "results_proof":   ("v5 guaranteed (widening on demand) + optimizations", "Proof campaign: final, e2e, e2e_nolog, nowiden, final_opt/e2e_opt (giveUp on), final_opt2/e2e_opt2 (previous recommendation), fin / fin_be (FINAL algorithm, profile final, with and without receipts)"),
    "results_final4":  ("FINAL (profile final)", "Main campaign on the final code: 0 lost of 332,624 (v4 reused)"),
    "results_sota2":   ("FINAL vs state of the art", "Same configs as results_sota: final, final_be, Le & Ngo, full GPSR (bbBFS/bbFlooding in results_sota)"),
    "results_cmp":     ("v5 modes", "Lab-baseline configs: v5, v5opt, v5guar (anycast)"),
    "results_sota":    ("v5 modes vs state of the art", "Identical configs: v4, v5, v5opt, v5guar, Le & Ngo, GPSR, bbBFS, bbFlooding; anycast + unicast; no/5 %/10 % failures (3 seeds) and root failures"),
}
COLS = ["campaign", "variant", "run", "group", "shape", "traffic", "failures", "timing", "N", "killed", "packets", "deliverable", "delivered", "pdr_pct",
        "reachCovMin", "dataMsgs", "ackMsgs", "ctlMsgs", "hbMsgs", "advMsgs", "mapMsgs", "mapdelMsgs", "splitMsgs", "totalMsgs", "dataBytes", "ackBytes", "ctlBytes",
        "avgHops", "stretch", "avgLatency_ms", "p95Latency_ms", "maxLatency_ms", "avgPeakBufB", "maxPeakBufB", "e2eResent", "resentDeliverable", "widens",
        "salvaged", "parkExpired", "logFull", "falsePos", "split", "inreg", "e2e", "hbAdapt", "baseline", "algo", "dupDeliveries", "dupDiscards", "linkRetx", "unreachStop", "wall_s"]

def parse(path):
    t = open(path, errors="ignore").read()
    d = dict(re.findall(r"(\w+)=([^\s]+)", t))
    m = re.search(r"WALL ([0-9.]+)", t)
    if m: d["wall_s"] = m.group(1)
    return d

rows = []
for camp in sorted(CAMPAIGNS):
    base = os.path.join(B, camp)
    if not os.path.isdir(base): continue
    files = glob.glob(f"{base}/**/*.txt", recursive=True)
    for f in sorted(files):
        d = parse(f)
        if not d or ("delivered" not in d and "reachCovMin" not in d and "totalMsgs" not in d): continue
        name = os.path.basename(f)[:-4]
        rel = os.path.relpath(os.path.dirname(f), base)
        p = name.split("__")
        variant = rel if rel != "." else (p[-1] if len(p) >= 4 else "")
        r = {k: d.get(k, "") for k in COLS}
        r.update(campaign=camp, variant=variant, run=name, group=p[0] if p else "",
                 shape=p[1] if len(p) > 1 else "", traffic=d.get("traffic", p[2] if len(p) > 2 else ""))
        tail = p[3:] if len(p) > 3 else []
        r["failures"] = next((x for x in tail if re.match(r"(none|r\d+s\d+|rootfail|s\d+|block|loss\d|slowhb|heavy)", x)), "")
        r["timing"] = next((x for x in tail if x in ("mid", "early", "late", "casc1", "casc3", "casc15", "fast", "waves", "wall")), "")
        try:
            dl, dv = float(d.get("deliverable") or 0), float(d.get("delivered") or 0)
            if dl: r["pdr_pct"] = f"{100 * min(dv, dl) / dl:.4f}"
        except ValueError: pass
        if not r["totalMsgs"]:
            try: r["totalMsgs"] = str(int(sum(float(d.get(k) or 0) for k in ("dataMsgs", "ackMsgs", "ctlMsgs"))))
            except ValueError: pass
        rows.append(r)

with open(os.path.join(B, "all_runs.csv"), "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=COLS); w.writeheader(); w.writerows(rows)
with open(os.path.join(B, "campaigns.csv"), "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["folder", "version", "what", "runs"])
    for c, (v, what) in CAMPAIGNS.items(): w.writerow([c, v, what, sum(1 for r in rows if r["campaign"] == c)])
print(len(rows), "runs ->", os.path.join(B, "all_runs.csv"))
