# RDRM final fixes – log

Dev suite: 14 hard runs from the proof campaign (cascades, 1 % and 5 % link loss, slow heartbeats, 3D maze geocast + unicast, ring),
guaranteed + adaptive heartbeats. Baseline = e2e_opt2 settings. Tool: tools/devsuite.py; outputs dev_<label>.txt and dev/<label>/.

| Step | Setting(s) | Lost (dev suite) | Duplicates | Messages | Bytes | Kept? |
|---|---|---|---|---|---|---|
| baseline | e2e_opt2 | 178 (all at 5 % loss) | 605 | 4.10 M | 35.3 M | – |
| link confirmations v1 | linkAck | 45 | 212 | 4.90 M | 53.3 M | refined |
| v2: compressed reports, gap wait | + gapWait | 69 | 219 | 4.94 M | 51.7 M | refined |
| v3: unlogged messages not numbered; stop waiting for old gaps | | 95 | 213 | 4.25 M | 38.5 M | refined |
| v4: + falseFix (re-add falsely deleted cells, versioned map news; wait longer on flaky links), hbScale | linkAck falseFix hbScale | 1 | 213 | 4.29 M | 39.0 M | refined |
| falseFix + hbScale alone (no linkAck) | | 51 (p95 69 s at 5 % loss) | 605 | 4.10 M | 35.2 M | no: linkAck needed |
| v5: reports ride on data going back | | 8 | 202 | 4.13 M | 39.0 M | refined |
| + goneRetries=2 | | 4 | 202 | 4.28 M (+3.2 % without loss) | 41.1 M | dropped (cost) |
| + goneWait=2 s (answer "gone" only for cells erased > 2 s ago) | goneWait | 1 | 202 | 4.26 M (non-loss 3.30 M < baseline 3.33 M) | 42.9 M | **kept** |
| gcKeep 12 s / splitWait 10 s for the geocast corridor case | | coverage 0.448 | | | | no effect, dropped |
| gcAck: geocast copies for other pieces acknowledged, entry module resends | gcAck | coverage 0.448 -> **1.000** | | data +0.4 % | | **kept** |

Root causes found on the way:
- Copies resent after a death were mostly already passed on (60 % of salvaged copies were duplicates).
- Under link loss, a neighbour wrongly declared dead stayed erased from the region map forever (no re-add): packets to that area expired.
- Messages that are not logged consumed link sequence numbers: one lost on the wire stalled the receiver's counter and triggered mass resends.
- Crossing "dead"/"alive again" floods flipped maps forever without version numbers.
- A geocast copy for another piece expired while the tear was still undetected (idle-link detection up to 7 s with adaptive heartbeats).
| unreach: "unreachable" after waiting for a piece not heard at full reach; source stops after 2 answers >= 16 s apart | unreach | 0 | 202 | 4.24 M | 42.1 M | **kept** (resends −17 %) |
| first resend 5 s instead of 3 s | rto=5 s | 0 | 125 (−38 %) | 4.16 M | 39.2 M | **kept** (p95 +0.04 s without loss) |
| first resend 2 s | rto=2 s | 0 | 277 | 4.16 M | 39.3 M | dropped |
| adaptive first resend (srtt + 4 rttvar, Karn, 2–6 s) | rtoAdapt | 0 | 208 | 4.20 M | 40.3 M | dropped (no gain) |
| idle-link heartbeat 1 s instead of 2 s | hbSlow=1 s | 0 | 97 | 4.86 M (+20 % without loss) | 37.0 M | dropped (p95 −24 % but +20 % messages): documented as a dial |
| full GPSR perimeter mode (face change on crossing the line Lp-D, stop on first edge again) | baseline=gpsr | same delivery | | data −25 to −42 % vs the simplified GPSR | | **kept** (fairer, stronger baseline) |
| final settings as code defaults | profile="final" (default); profile="legacy" = old defaults | legacy reproduces old runs exactly (v4 1500/1500, 25,362 data, 105,557 ctl); default = dev settings exactly | | | | **kept** |

## Final version (profile="final")
cti=poison hbScope=all leader=elect balance=detour inreg=adapt salvage=on suspect=on split=local widen=on
e2e=on (rto 5 s, x2, <= 16 s, 10 tries) ackCopy=off hbAdapt=on (idle 2 s, busy 0.5 s) hbScale=on
linkAck=on (per-link numbers; reports on heartbeats/data: passed on / received) falseFix=on (re-add, versioned, +3 misses on flaky links)
goneWait=2 s gcAck=on unreach=on
Best-effort mode: same with e2e="off".

## Final campaigns (launched)
- results_proof/fin, results_proof/fin_be: 2D timing (590), adversarial (63), ring (3), 3D (190) each
- results_final4: main campaign (v4 reused)
- results_sota2: final, final_be, Le & Ngo, full GPSR (BFS / flooding reused from results_sota)

## Found during the final proof campaign: anycast duplicates at the same module
3D anycast (fin): 3,172 duplicate deliveries in 28 runs, almost all from end-to-end resends whose receipt was lost: the resent copy went back to the module that had already delivered it.
Fix anyDedup=on: a module remembers what it delivered for anycast too, drops a later copy and answers again (no change to routing or message count).
| test | dup deliveries before | after (dropped at the module) | delivered | data msgs |
|---|---|---|---|---|
| D maze3 anycast s3 | 233 | 30 (203) | 5065 = | 178,190 = |
| D cavity anycast s3 | 199 | 15 (184) | 5380 = | 90,813 = |
**kept**, part of profile final. Binary swapped atomically during the campaign (06:26); anycast runs made before the swap are rerun.

## Found during the final proof campaign: gradients stuck after a lost advert (link loss)
Lossy runs kept 40-518 packets that were never even sent (tx=0): the source's region had no gradient to the target region for the rest of the run (even with a 120 s tail).
Cause: gradient adverts are sent once, on change; a lost advert, or a neighbour falsely declared dead and taken back, leaves the stale distance forever (the taken-back neighbour never resends its own adverts).
Fix advSync=on: every 4th heartbeat carries a 1-byte checksum of the distances advertised on that link; if it does not match what the neighbour holds (and no advert arrived in the last 2 periods), it asks once (1-byte request, at most 1/s) and gets all distances again.
| test | lost before | lost after | notes |
|---|---|---|---|
| A swiss loss1 s5 | 40 / 952 | 0 | 114 re-syncs |
| A holes loss5 s3 | 91 / 1105 | 0 | 859 re-syncs, data 377,729 -> 260,479 |
| dev suite (14 hard runs) | 0 lost, 4.155M msgs, 39.24 MB | 0 lost, 4.104M msgs, 37.64 MB | cheaper overall |
**kept**, part of profile final. All final campaigns rerun with it (the first final runs are kept in results_preadvsync/).

## Final campaigns (final code, advSync included) — results
Proof campaign (results_proof/fin, fin_be; 840 runs each):
| test | fin | fin_be | previous guaranteed (e2e_opt2) | best effort, normal hb (final) |
|---|---|---|---|---|
| 2D timing 590 runs, 941,132 deliverable | 0 lost | 441 | 0 | 0 |
| 3D 190 runs, 803,375 | 0 lost | 135 | 0 | 19 |
| adversarial no loss, 42,097 | 0 lost | 136 | 0 | 0 |
| ring, 4,221 | 0 lost | 0 | 0 | 0 |
| geocast worst 2D/3D | 1.000/1.000 | 0.000/0.448 | 1.000/0.448 | 1.000/1.000 |
| msgs/run 2D/3D | 252,059/906,020 | 175,832/726,025 | 270,010/920,663 | 332,124/1,596,048 |
| dup 2D/3D | 1,231/516 | 545/176 | 47,035/9,438 | 32,870/6,493 |
| 1 % loss lost | 4 | 384 | 315 | 3,711 |
| 5 % loss lost | 157 | 1,871 | 5,815 | 12,393 |
Main campaign (results_final4): final 332,624/332,624, 84/84 perfect (v4 98.47 %); geocast 100 %; loss 99.75-100 %.
SOTA (results_sota2): final 100 % everywhere; anycast 63.4 msgs/pkt (final_be 42.1), unicast 87.1 (59.4); GPSR full 99.6-100 % at 180-285; Le & Ngo 91-99 %.

## Ablation campaign
Same 846 configurations as results_proof/fin, final code, layers removed one at a time (all without receipts):
- abl_dv: e2e=off salvage=off split=none widen=off gcAck=off  (plain per-region distance vector + heartbeats + poisoning)
- abl_notorn: e2e=off split=none widen=off gcAck=off          (no torn-region handling)
- abl_nosalv: e2e=off salvage=off                             (no per-link recovery)
fin_be (e2e=off, everything else on) and fin (full) already exist.
Results (timing + 3D, 780 runs per variant; ablation_tables.txt):
| variant | 2D lost / 941,132 | 3D lost / 803,375 | geocast worst | msgs/run 2D / 3D |
|---|---|---|---|---|
| fin (full) | 0 | 0 | 1.000 | 252,059 / 906,020 |
| fin_be (no receipts) | 441 | 135 | 0.000 | 175,832 / 726,025 |
| abl_notorn (no receipts, no torn handling) | 5,573 | 4,889 | 0.000 | 169,565 / 700,068 |
| abl_nosalv (no receipts, no salvage) | 86,168 | 34,304 | 0.000 | 164,280 / 708,421 |
| abl_dv (plain per-region DV) | 90,758 | 38,399 | 0.000 | 159,406 / 684,557 |
Each layer pays for itself; salvage is the largest.
