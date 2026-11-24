# RADRA pseudocode

RADRA (Region-Aware Distributed Routing Algorithm) runs the same code on every module.
The paper gives Algorithm 2 (forwarding and delivery). This page gives all three
algorithms, with a short explanation of each part. The implementation is in
`app/bbRDRM5/` (`bbRDRM5` is the historical code name of the app).

## Notation

| Symbol | Meaning |
|---|---|
| `i` | the module running the code |
| `ρ(i)` | region of module `i` |
| `N(i)`, `N_a(i)` | physical neighbours of `i`, and those currently believed alive |
| `R` | set of regions |
| `D[r]` | distance (in hops) from `i` to the nearest module of region `r` |
| `D_n[j][r]` | distance to `r` last advertised by neighbour `j` |
| `D^π` | gradient toward one piece `π` of a torn region |
| `M` | map of the cells of `i`'s own region |
| `p_r(j)` | penalty of neighbour `j` toward `r` (load, energy, suspicion) |
| `λ` | penalty margin for a detour; `p.det` the detours left to packet `p` |
| `T_h`, `T_hb(j)` | hold-down delay; heartbeat period of `j` |
| `W`, `Got` | waiting room for packets with no next hop; packets already delivered |

Packet fields: `p.r` (target region), `p.mode` (ANYCAST, UNICAST or GEOCAST),
`p.dst` (target cell for unicast), `p.pid` (packet id), `p.ttl` (hop limit),
`p.lseq` (link sequence number).

---

## Algorithm 1: region gradients, construction and maintenance

```
Require: ρ(i), N(i), regions R, hold-down T_h
Ensure:  D[r], advertised distances D_n[j][r], hold[r], N_a(i) ← N(i)

Init:
  for all r in R: D[r] ← 0 if r = ρ(i) else ∞
  send Adv_Msg(ρ(i), 0) to all j in N(i)

On Adv_Msg(r, d_sent) from j:
  D_n[j][r] ← d_sent
  Recompute(r)

procedure Recompute(r):
  if r ≠ ρ(i) and now ≥ hold[r]:
    d ← 1 + min over j in N_a(i) of D_n[j][r]
    if d < D[r]:                                   # good news: spread at once
      D[r] ← d;  send Adv_Msg(r, d) to all j in N_a(i)
    else if d > D[r]:                              # bad news: poison, then wait
      D[r] ← ∞;  hold[r] ← now + T_h
      send Adv_Msg(r, ∞) to all j in N_a(i)
      schedule Recompute(r) at hold[r]

On Hb_Msg(load, energy, rep, σ) from j:
  t_last[j] ← now;  Q_j ← load;  F_j ← energy
  LinkReport(j, rep)                               # Algorithm 3
  if σ ≠ ⊥ and σ ≠ Checksum(D_n[j]):
    send AdvReq_Msg to j                           # an advert was lost

On AdvReq_Msg from j:
  for all r in R: send Adv_Msg(r, D[r]) to j

When now − t_last[j] > 3·T_hb(j):                  # j is silent: declared dead
  N_a(i) ← N_a(i) \ {j};  D_n[j][·] ← ∞
  for all r in R: Recompute(r)
  if ρ(j) = ρ(i): send MapDel_Msg(cell(j)) to the region neighbours
  Salvage(j)                                       # Algorithm 3
```

**How it works.**
- *Construction.* Every module of region `r` starts at distance 0, so each region is
  the source of a multi-source gradient: a module's `D[r]` is its distance to the
  closest module of `r`. Each module keeps one distance per region, not one per module.
- *Good and bad news.* A shorter distance is spread at once. A longer one means the old
  path broke. The module then **poisons** the region (advertises ∞) and waits `T_h`
  before accepting a new value. Stale distances are flushed before anyone relies on them,
  which removes counting to infinity: the repair stays local and ends in a bounded time.
- *Failure detection.* Modules can fail silently, so each module sends heartbeats. A
  neighbour silent for three periods is declared dead. Its distances are dropped, the
  gradients are recomputed, its cell is removed from the region map, and the packets
  it was holding are salvaged.
- *Lost adverts.* A heartbeat carries a checksum of the sender's distance table. A
  mismatch makes the receiver ask for the table again, so a lost `Adv_Msg` cannot leave
  a stale distance for ever.

---

## Algorithm 2: region-based forwarding and delivery

```
Require: D, D_n, N_a(i), region map M, piece gradients D^π
Ensure:  delivered set Got, waiting room W

On Data_Msg(p) from j:
  if not LinkReceive(j, p): return                 # duplicate on the link
  p.ttl ← p.ttl − 1
  if p.ttl > 0: Forward(p)

procedure Forward(p):
  if ρ(i) = p.r: Deliver(p); return
  g ← D^π if p.mode = UNICAST and p.dst is in a known piece π of p.r, else D
  C ← { j in N_a(i) | g_n[j][p.r] < g[p.r] }       # neighbours one hop closer
  if C = ∅:
    W ← W ∪ {p};  retry after Δ_w                  # gradient under repair
  else:
    j* ← argmin over j in C of p_{p.r}(j)          # suspects excluded, with hysteresis
    if p_{p.r}(j*) ≥ λ and p.det > 0 and a neighbour k with g_n[k][p.r] = g[p.r]
       has p_{p.r}(k) + λ < p_{p.r}(j*):
      j* ← k;  p.det ← p.det − 1                   # bounded sideways detour
    LinkSend(j*, p)                                # Algorithm 3

procedure Deliver(p):
  if p.mode = ANYCAST or (p.mode = UNICAST and cell(i) = p.dst):
    if p.pid not in Got: Got ← Got ∪ {p.pid}; hand p to the application
    Receipt(p, OK)                                 # also for a duplicate
  else if p.mode = UNICAST:
    if p.dst not in M for more than T_gone: Receipt(p, GONE)
    else if p.dst is in another piece π: follow D^π as in Forward
    else: LinkSend(j, p), j = next cell of a shortest path to p.dst in M
  else if p.pid not in Got:                        # GEOCAST
    Got ← Got ∪ {p.pid};  hand p to the application
    send p to all j in N_a(i) with ρ(j) = p.r      # flood inside the region only
    if i is the entry module:
      Receipt(p, OK);  send one copy to each other piece
```

**How it works.**
- *Forwarding.* A packet goes to a neighbour whose distance to the target region is
  smaller. All such neighbours lie on a shortest path, so the choice among them is free.
  RADRA uses it to pick the neighbour with the lowest penalty: a busy, low-energy or
  suspected module is avoided without lengthening the path.
- *Detours.* If even the best next hop is overloaded, a packet may take a bounded number
  of sideways steps (same distance) to a clearly less loaded neighbour. This spreads a
  hotspot over parallel paths.
- *Waiting room.* While a gradient is being repaired, a module may have no closer
  neighbour. It keeps the packet and retries, instead of dropping it.
- *Delivery.* Anycast stops at the first module of the region. Unicast continues inside
  the region on the region map to the target cell. Geocast enters the region once, then
  floods only inside it. Duplicates are suppressed with `Got`, and duplicates are still
  acknowledged so the source stops resending.
- *Torn regions.* When failures cut a region into pieces, each piece builds its own
  gradient `D^π`. A unicast packet follows the gradient of the piece that holds its
  target cell, and a geocast packet sends one copy to each piece.

---

## Algorithm 3: per-link recovery and end-to-end receipts

```
Require: RTO_0 = 5 s, RTO_max = 16 s, gap delay T_gap, wait limit T_w
Ensure:  link counters seq[j], copy logs Log[j], pending table Pend

procedure LinkSend(j, p):
  seq[j] ← seq[j] + 1;  p.lseq ← seq[j]
  Log[j] ← Log[j] ∪ {p}                            # keep a copy until j passes it on
  send Data_Msg(p) to j

procedure LinkReport(j, rep):                       # rep comes in j's heartbeat
  remove from Log[j] the packets rep reports as passed on
  resend those missing for more than T_gap

procedure Salvage(j):                              # j was declared dead
  for all p in Log[j]: Forward(p)                  # re-route on the repaired gradients
  Log[j] ← ∅

procedure Send(p):                                 # at the source
  Pend[p.pid] ← (p, rto ← RTO_0, tries ← 1);  arm the timer of p.pid
  Forward(p)

When the timer of pid expires and Pend[pid].tries < 10:
  Forward(Pend[pid].p)
  rto ← min(2·rto, RTO_max);  tries ← tries + 1;  re-arm the timer

On Ack_Msg(pid, kind) at the source:
  if kind in {OK, GONE}: remove Pend[pid]
  if kind = UNREACH and an earlier UNREACH for pid is older than RTO_max:
    remove Pend[pid]

When p in W waits more than T_w for a piece π not announced:
  if reach covers the robot: Receipt(p, UNREACH)
  else: send Widen_Msg(p.r, 2·reach)               # pieces re-announce up to 2·reach hops
```

**How it works.**
- *Per-link recovery.* Each module keeps a copy of every packet it sends until the
  next module reports, in its heartbeat, that it has passed the packet on. A packet
  missing from the report is resent after `T_gap`. When a neighbour dies, every packet
  it was holding is re-routed from the copies (**salvage**), so a silent failure does not
  swallow the packets in flight.
- *End-to-end receipts.* The source keeps each packet until it gets a receipt, and
  resends it with exponential back-off (5 s, 10 s, then 16 s; at most 10 tries).
  `OK` means delivered. `GONE` means the target cell is dead. `UNREACH`, confirmed
  twice, means no piece of the target region is reachable.
- *Widening.* Piece announcements start local. If a packet waits for a piece that was
  never announced to it, the search radius is doubled until it covers the robot.
  The cost of tearing therefore stays near the cut.

Together, these give the property of the paper: every packet whose source and target
stay alive and connected is delivered, at a bounded message cost.
