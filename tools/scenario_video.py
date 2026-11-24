"""Slow-motion videos of real RADRA runs, rendered from the VisibleSim event log (RR_LOG).

Every dot is a real message of the run: each hop is a `tx` event at the sender
followed by a `visit` event at the receiver. Modules, failures, detections, repairs,
salvages, detours and deliveries are all taken from the same log. A panel on the right
gives live counters computed from the log and explains what is happening.

usage: scenario_video.py <scenario> <out.mp4> [logdir] [cfgdir]
scenarios: anycast unicast geocast failures hotspot torn
logdir/cfgdir hold <name>.csv / <name>.xml (default: current directory).
"""
import sys, os, csv, re, bisect, collections, shutil, subprocess
from PIL import Image, ImageDraw, ImageFont

ACK = 1 << 24
FPS = 25
W, H = 1920, 1080
PANEL_X = 1330
RC = {1: (232, 197, 71), 2: (127, 166, 224), 3: (124, 192, 138), 4: (109, 193, 191), 5: (224, 130, 110), 6: (181, 138, 209)}
RNAME = {1: "head", 2: "torso", 3: "left arm", 4: "right arm", 5: "left leg", 6: "right leg"}
NAVY, GREEN, ORANGE, MAGENTA, PURPLE = (20, 40, 110), (20, 150, 60), (235, 120, 0), (200, 0, 150), (110, 60, 200)
GREY, INK, SOFT = (90, 90, 90), (20, 20, 20), (120, 120, 120)


def font(sz, bold=False):
    for f in (["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"] if bold else []) + ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
        if os.path.exists(f):
            return ImageFont.truetype(f, sz)
    return ImageFont.load_default()


FT, FH, F, FS, FXS = font(34, True), font(24, True), font(21), font(18), font(15)


def mix(c, w, a):
    return tuple(int(c[i] * (1 - a) + w[i] * a) for i in range(3))


def ylorrd(x):
    st = [(255, 255, 204), (254, 217, 118), (253, 141, 60), (227, 26, 28), (128, 0, 38)]
    x = max(0, min(1, x)) * 4
    i = min(int(x), 3)
    return mix(st[i], st[i + 1], x - i)


class Robot:
    def __init__(self, cfg):
        self.pos, self.reg = {}, {}
        k = 0
        for line in open(cfg):
            m = re.search(r'<block position="(\d+),(\d+),(\d+)"', line)
            if not m:
                continue
            k += 1
            self.pos[k] = (int(m[1]), int(m[3]))
            r = re.search(r'region="(\d+)"', line)
            self.reg[k] = int(r[1]) if r else 1
        self.GW = max(x for x, _ in self.pos.values()) + 1
        self.GH = max(z for _, z in self.pos.values()) + 1
        self.at = {v: k for k, v in self.pos.items()}

    def place(self, x0, y0, w, h):
        self.cell = int(min(w / self.GW, h / self.GH))
        self.ox = x0 + (w - self.cell * self.GW) // 2
        self.oy = y0 + (h - self.cell * self.GH) // 2

    def xy(self, m):
        x, z = self.pos[m]
        return self.ox + x * self.cell, self.oy + (self.GH - 1 - z) * self.cell

    def c(self, m):
        x, y = self.xy(m)
        return x + self.cell / 2, y + self.cell / 2

    def adj(self, a, b):
        (x1, z1), (x2, z2) = self.pos[a], self.pos[b]
        return abs(x1 - x2) + abs(z1 - z2) == 1


class Run:
    """One simulation log, turned into hop segments and per-event lists."""

    def __init__(self, robot, log):
        self.R = robot
        ev = collections.defaultdict(list)
        txs = collections.defaultdict(list)
        visits = []
        self.send, self.dst, self.region_of_pid = {}, {}, {}
        for r in csv.DictReader(open(log)):
            t, e, i, p, v = int(r['t']), r['ev'], int(r['id']), int(r['pid']), int(r['v'])
            if e == 'tx' or e == 'acktx':
                txs[p].append((t, i))
            elif e == 'visit':
                visits.append((t, i, p))
            elif e == 'send':
                self.send[p] = (t, i)
                self.region_of_pid[p] = v
            elif e == 'sdst':
                self.dst[p] = i
            if e not in ('visit', 'tx', 'acktx'):
                ev[e].append((t, i, p, v))
        self.ev = ev
        segs = []
        for t, i, p in visits:                       # pair each arrival with its sender (adjacent, latest tx before it)
            best = None
            for tt, s in reversed(txs.get(p, [])):
                if tt > t:
                    continue
                if t - tt > 60000:
                    break
                if s in robot.pos and i in robot.pos and robot.adj(s, i):
                    best = (tt, s)
                    break
            if best:
                segs.append((best[0], t, best[1], i, p))
        segs.sort()
        self.segs = segs
        self.seg_t0 = [s[0] for s in segs]
        self.maxlen = max((s[1] - s[0] for s in segs), default=10000)
        self.tx_sorted = sorted((t, i, p) for p, l in txs.items() for t, i in l)
        self.tx_t = [x[0] for x in self.tx_sorted]
        self.all_ev = sorted((t, e, i, p, v) for e, l in ev.items() for t, i, p, v in l)
        self.all_t = [x[0] for x in self.all_ev]

    def active(self, T):
        lo = bisect.bisect_left(self.seg_t0, T - self.maxlen)
        hi = bisect.bisect_right(self.seg_t0, T)
        return [s for s in self.segs[lo:hi] if s[1] >= T]

    def recent(self, T, span):
        lo = bisect.bisect_left(self.seg_t0, T - span - self.maxlen)
        hi = bisect.bisect_right(self.seg_t0, T)
        return [s for s in self.segs[lo:hi] if T - span <= s[1] < T]


class State:
    """Counters accumulated as the video advances through the log."""

    def __init__(self, run):
        self.run = run
        self.k_ev = 0
        self.k_tx = 0
        self.sent = self.data_hops = self.rcpt_hops = self.ctl = 0
        self.delivered = set()
        self.lat = []
        self.killed, self.detected = {}, {}
        self.salv = self.expired = self.drops = self.resent = self.detours = self.widen = self.ctl_at_fail = 0
        self.recovered = set()
        self.load = collections.Counter()
        self.flash = []                              # (frame_until, module, kind)
        self.ctlheat = collections.Counter()
        self.ctlq = collections.deque()
        self.lefthits = 0

    def advance(self, T, dt, frame, cfg):
        run = self.run
        while self.k_tx < len(run.tx_sorted) and run.tx_sorted[self.k_tx][0] <= T:
            t, i, p = run.tx_sorted[self.k_tx]
            self.k_tx += 1
            if p >= ACK:
                self.rcpt_hops += 1
            else:
                self.data_hops += 1
                self.load[i] += 1
        win = max(3 * dt, 150000)
        while self.ctlq and self.ctlq[0][0] < T - win:
            _, m = self.ctlq.popleft()
            self.ctlheat[m] -= 1
            if self.ctlheat[m] <= 0:
                del self.ctlheat[m]
        while self.k_ev < len(run.all_ev) and run.all_ev[self.k_ev][0] <= T:
            t, e, i, p, v = run.all_ev[self.k_ev]
            self.k_ev += 1
            fresh = t >= T - 2 * dt
            self.fl = 1 if dt <= 20000 else 0
            if e == 'send':
                self.sent += 1
                fresh and self.flash.append((frame + self.fl * 5, i, 'send'))
            elif e == 'deliver' and p < ACK:
                if p not in self.delivered:
                    self.delivered.add(p)
                    if p in run.send:
                        self.lat.append(t - run.send[p][0])
                    if cfg.get('left') and i in cfg['left'] and t > 15e6:
                        self.lefthits += 1
                fresh and self.flash.append((frame + self.fl * 7, i, 'deliver'))
            elif e == 'ctl':
                self.ctl += 1
                if t >= T - max(3 * dt, 150000):
                    self.ctlq.append((t, i))
                    self.ctlheat[i] += 1
            elif e == 'kill':
                if not self.killed:
                    self.ctl_at_fail = self.ctl
                self.killed[i] = t
            elif e == 'dead':
                if p not in self.detected:
                    self.detected[p] = t
                fresh and self.flash.append((frame + self.fl * 10, i, 'detect'))
            elif e == 'salvage':
                self.salv += 1
                self.recovered.add(p)
                fresh and self.flash.append((frame + self.fl * 8, i, 'salvage'))
            elif e == 'expire':
                self.expired += 1
            elif e == 'drop':
                self.drops += 1
                fresh and self.flash.append((frame + self.fl * 8, i, 'drop'))
            elif e == 'e2eresend':
                self.resent += 1
                self.recovered.add(p)
                fresh and self.flash.append((frame + self.fl * 8, i, 'resend'))
            elif e == 'detour':
                self.detours += 1
                fresh and self.flash.append((frame + self.fl * 6, i, 'detour'))
            elif e == 'widen':
                self.widen += 1
        self.flash = [f for f in self.flash if f[0] >= frame]


    def pct(self, q):
        if not self.lat:
            return 0
        s = sorted(self.lat)
        return s[min(len(s) - 1, int(q * len(s)))] / 1e6


def draw_robot(d, R, st, T, frame, mode):
    """mode: 'region' (region colours) or 'load' (data messages relayed per module)."""
    for m in R.pos:
        x, y = R.xy(m)
        c = R.cell
        if m in st.killed and st.killed[m] <= T:
            if m in st.detected:
                d.rectangle([x + 1, y + 1, x + c - 2, y + c - 2], fill=(25, 25, 25))
            else:
                d.rectangle([x + 1, y + 1, x + c - 2, y + c - 2], fill=(70, 70, 70), outline=(220, 30, 30), width=2)
            continue
        if mode == 'load':
            col = ylorrd(st.load.get(m, 0) / 700)
        else:
            col = mix(RC.get(R.reg[m], (200, 200, 200)), (255, 255, 255), 0.45)
        h = min(1, st.ctlheat.get(m, 0) / 3)
        if h > 0.05:
            col = mix(col, PURPLE, 0.8 * h)
        d.rectangle([x + 1, y + 1, x + c - 2, y + c - 2], fill=col)
    for until, m, kind in st.flash:
        if m not in R.pos:
            continue
        x, y = R.xy(m)
        c = R.cell
        if kind == 'deliver':
            d.rectangle([x - 1, y - 1, x + c, y + c], outline=GREEN, width=3)
        elif kind == 'detect':
            d.ellipse([x - 3, y - 3, x + c + 2, y + c + 2], outline=(240, 190, 0), width=3)
        elif kind == 'salvage' or kind == 'resend':
            d.rectangle([x - 1, y - 1, x + c, y + c], outline=ORANGE, width=3)
        elif kind == 'drop':
            d.line([x + 2, y + 2, x + c - 3, y + c - 3], fill=(220, 30, 30), width=3)
            d.line([x + 2, y + c - 3, x + c - 3, y + 2], fill=(220, 30, 30), width=3)
        elif kind == 'detour':
            d.rectangle([x, y, x + c - 1, y + c - 1], outline=MAGENTA, width=2)
        elif kind == 'send':
            d.rectangle([x + 2, y + 2, x + c - 3, y + c - 3], outline=NAVY, width=2)


def draw_packets(d, R, run, st, T, dt, gcfill=False):
    r = max(3, R.cell * 0.32)
    if gcfill:                                              # modules the geocast flood reached in the last 0.15 s
        for t0, t1, a, b, p in run.recent(T, max(dt, 150000)):
            if p < ACK and b in R.pos and R.reg[b] == run.region_of_pid.get(p):
                x, y = R.xy(b)
                a_ = 0.35 + 0.65 * (1 - (T - t1) / max(dt, 150000))
                d.rectangle([x + 1, y + 1, x + R.cell - 2, y + R.cell - 2], fill=mix(RC[R.reg[b]], (60, 60, 60), 0.45 * a_))
    for t0, t1, a, b, p in run.recent(T, max(dt, 9000)):     # short tail of the previous hop
        if a in R.pos and b in R.pos:
            ack = p >= ACK
            col = mix(GREEN if ack else (ORANGE if p in st.recovered else NAVY), (255, 255, 255), 0.55)
            d.line([R.c(a), R.c(b)], fill=col, width=2 if ack else 3)
    for t0, t1, a, b, p in run.active(T):
        if a not in R.pos or b not in R.pos:
            continue
        f = (T - t0) / max(1, t1 - t0)
        (xa, ya), (xb, yb) = R.c(a), R.c(b)
        x, y = xa + (xb - xa) * f, ya + (yb - ya) * f
        if p >= ACK:
            rr = r * 0.62
            d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=GREEN)
        else:
            col = ORANGE if p in st.recovered else NAVY
            ring = RC.get(run.region_of_pid.get(p, 0), (255, 255, 255))
            d.ellipse([x - r - 2, y - r - 2, x + r + 2, y + r + 2], fill=ring)
            d.ellipse([x - r, y - r, x + r, y + r], fill=col)


def wrap(d, text, fnt, width):
    out = []
    for para in text.split("\n"):
        line = ""
        for w in para.split(" "):
            t = (line + " " + w).strip()
            if d.textlength(t, font=fnt) <= width:
                line = t
            else:
                out.append(line)
                line = w
        out.append(line)
    return out


def speed_label(dt):
    s = dt * FPS / 1e6
    if s < 0.999:
        return f"slow motion  ×{s:.2f}", (30, 90, 200)
    if s < 1.001:
        return "real time  ×1", GREY
    return f"fast forward  ×{s:.0f}", (190, 90, 0)


def panel(d, spec, T, dt, rows, caption, legend, y0=40):
    X, Wp = PANEL_X, W - PANEL_X - 40
    d.rectangle([X - 20, 0, W, H], fill=(246, 247, 250))
    d.line([X - 20, 0, X - 20, H], fill=(220, 222, 228), width=2)
    y = y0
    for ln in wrap(d, spec['title'], FH, Wp):
        d.text((X, y), ln, fill=INK, font=FH)
        y += 32
    for ln in wrap(d, spec['sub'], FXS, Wp):
        d.text((X, y), ln, fill=SOFT, font=FXS)
        y += 20
    y += 14
    d.text((X, y), f"t = {T / 1e6:6.3f} s", fill=INK, font=FH)
    lab, col = speed_label(dt)
    tw = d.textlength(lab, font=FS)
    d.rounded_rectangle([X + Wp - tw - 24, y, X + Wp, y + 30], radius=8, fill=col)
    d.text((X + Wp - tw - 12, y + 4), lab, fill='white', font=FS)
    y += 48
    for a, b in rows:
        if a is None:
            y += 8
            d.line([X, y, X + Wp, y], fill=(215, 218, 225), width=1)
            y += 10
            continue
        d.text((X, y), a, fill=GREY, font=FS)
        bw = d.textlength(b, font=F)
        d.text((X + Wp - bw, y - 2), b, fill=INK, font=F)
        y += 29
    y += 12
    d.rounded_rectangle([X - 6, y, X + Wp + 6, y + 8 + 26 * 7], radius=10, fill=(232, 238, 250))
    d.text((X + 6, y + 8), "What is happening", fill=(30, 60, 140), font=FS)
    yy = y + 36
    for ln in wrap(d, caption, FS, Wp - 16)[:6]:
        d.text((X + 6, yy), ln, fill=INK, font=FS)
        yy += 25
    y += 8 + 26 * 7 + 18
    for kind, lab in legend:
        cx, cy = X + 11, y + 11
        if kind == 'data':
            d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], fill=(232, 197, 71))
            d.ellipse([cx - 7, cy - 7, cx + 7, cy + 7], fill=NAVY)
        elif kind == 'rcpt':
            d.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=GREEN)
        elif kind == 'recov':
            d.ellipse([cx - 7, cy - 7, cx + 7, cy + 7], fill=ORANGE)
        elif kind == 'deliver':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], outline=GREEN, width=3)
        elif kind == 'failed':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], fill=(70, 70, 70), outline=(220, 30, 30), width=2)
        elif kind == 'dead':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], fill=(25, 25, 25))
        elif kind == 'detect':
            d.ellipse([cx - 11, cy - 11, cx + 11, cy + 11], outline=(240, 190, 0), width=3)
        elif kind == 'repair':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], fill=PURPLE)
        elif kind == 'salv':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], outline=ORANGE, width=3)
        elif kind == 'drop':
            d.line([cx - 8, cy - 8, cx + 8, cy + 8], fill=(220, 30, 30), width=3)
            d.line([cx - 8, cy + 8, cx + 8, cy - 8], fill=(220, 30, 30), width=3)
        elif kind == 'detour':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], outline=MAGENTA, width=2)
        elif kind == 'gcin':
            d.rectangle([cx - 10, cy - 10, cx + 10, cy + 10], fill=mix(RC[2], (60, 60, 60), 0.45))
        elif kind == 'load':
            for k in range(5):
                d.rectangle([cx - 10 + k * 8, cy - 9, cx - 3 + k * 8, cy + 9], fill=ylorrd(k / 4))
            cx += 30
        d.text((cx + 24, y), lab, fill=(60, 60, 60), font=FXS)
        y += 27


def card(path, lines):
    im = Image.new('RGB', (W, H), 'white')
    d = ImageDraw.Draw(im)
    y = 330
    for fnt, txt, col in lines:
        for ln in wrap(d, txt, fnt, 1500):
            tw = d.textlength(ln, font=fnt)
            d.text(((W - tw) / 2, y), ln, fill=col, font=fnt)
            y += fnt.size + 16
        y += 10
    im.save(path)


def timeline(phases):
    """phases: [(t_start, t_end, sim_us_per_frame)] -> list of (T, dt)."""
    out = []
    for a, b, dt in phases:
        T = a
        while T < b:
            out.append((T, dt))
            T += dt
    return out


def caption_at(caps, T):
    c = caps[0][1]
    for t, txt in caps:
        if T >= t:
            c = txt
    return c


# ----------------------------------------------------------------------------- scenarios
def SPEC(name):
    slow, med, ff, vff = 2000, 20000, 200000, 600000
    if name in ('anycast', 'unicast'):
        any_ = name == 'anycast'
        return dict(
            runs=[name], mode='region', title=f"RADRA: {'anycast' if any_ else 'unicast'} traffic",
            sub="Real run of the final RADRA code in VisibleSim, 400-module humanoid, rendered from its event log.",
            intro=[(FT, f"RADRA in action: {'anycast' if any_ else 'unicast'}", INK),
                   (F, ("Every module sends one packet to each of the five other regions. "
                        "Anycast: the packet is delivered to the first module of the target region it reaches.") if any_ else
                       ("Every module sends one packet to one chosen module of each other region. "
                        "Unicast: the packet follows the region gradient to the region, then the region map to its target cell."), GREY),
                   (FS, "Each dot is a real message of the run. Slow motion: one second of video is 0.05 s of simulated time.", SOFT)],
            phases=[(4.995e6, 6.2e6, slow), (6.2e6, 8e6, med), (8e6, 45.4e6, ff)],
            caps=[(0, "Traffic starts. Each module keeps one distance per region. A packet always moves to a neighbour that is one hop closer to its target region, so it follows a shortest path."),
                  (5.25e6, "Several neighbours can be one hop closer. RADRA picks the least loaded one among them, so traffic spreads over equally short paths." if any_ else
                   "Outside the target region, unicast travels exactly like anycast. Once inside, the packet uses the region map to reach the one cell it is addressed to."),
                  (5.6e6, "When a packet arrives, a receipt (green) goes back to the source. The source stops resending once the receipt arrives."),
                  (8e6, "Fast forward: 2,000 packets over 40 s. Watch the counters: everything sent is delivered, at shortest-path cost.")],
            legend=[('data', 'data packet (ring = colour of target region)'), ('rcpt', 'end-to-end receipt going back'),
                    ('deliver', 'packet delivered here')],
            kind='basic', regionkey=True)
    if name == 'geocast':
        return dict(
            runs=['geocast'], mode='region', title="RADRA: geocast traffic",
            sub="Real run of the final RADRA code in VisibleSim, 400-module humanoid, rendered from its event log.",
            intro=[(FT, "RADRA in action: geocast", INK),
                   (F, "One module in five sends one packet to every module of another region. The packet travels to the region along the gradient as a single copy, then floods only inside that region.", GREY),
                   (FS, "Each dot is a real message of the run. Slow motion: one second of video is 0.05 s of simulated time.", SOFT)],
            phases=[(5.08e6, 6.4e6, slow), (6.4e6, 13.4e6, med)],
            caps=[(0, "A geocast leaves its source as one copy and follows the gradient toward the target region, like anycast."),
                  (5.14e6, "At the region border the entry module delivers it and starts a flood that stays inside the region: every module of the region receives it once and passes it to its region neighbours."),
                  (5.5e6, "Modules outside the target region never see the flood. One receipt (green) goes back from the entry module to the source."),
                  (6.4e6, "Faster (×0.5): 80 geocasts, each reaching a whole region.")],
            legend=[('data', 'geocast copy (ring = colour of target region)'), ('rcpt', 'end-to-end receipt going back'),
                    ('gcin', 'reached by the in-region flood'), ('deliver', 'geocast delivered at region entry')],
            kind='basic', regionkey=True, gcfill=True)
    if name == 'failures':
        return dict(
            runs=['failures'], mode='region', title="RADRA: 10 % of the modules fail silently",
            sub="Real run of the final RADRA code in VisibleSim, 400-module humanoid, anycast, 40 random modules stop at t = 15 s without warning.",
            intro=[(FT, "RADRA in action: silent failures", INK),
                   (F, "At t = 15 s, 40 random modules (10 %) stop. They send nothing and tell nobody. Watch the heartbeats detect them, the gradients repair locally, the held packets get salvaged, and receipts re-send what was lost.", GREY),
                   (FS, "Each dot is a real message of the run. The speed badge on the right shows when the video slows down.", SOFT)],
            phases=[(14.55e6, 15.15e6, slow), (15.15e6, 16.08e6, med), (16.08e6, 17.3e6, slow * 2), (17.3e6, 23e6, med * 2), (23e6, 50e6, vff)],
            caps=[(0, "Traffic is flowing normally before the failure."),
                  (15.0e6, "t = 15 s: 40 modules stop silently (red outline). Packets sent to them simply vanish, but every sender kept a copy of each packet until the next module confirms it."),
                  (16.1e6, "From about 1.2 s later, neighbours that miss three heartbeats declare the module dead (yellow ring) and it turns black. All 40 are detected by t ≈ 22 s."),
                  (16.35e6, "Repair: the neighbours poison the broken distances and re-advertise (purple). Only modules whose distance changed take part; most of the arms and legs stay quiet."),
                  (16.6e6, "Salvage: the copies held for dead neighbours are re-routed on the repaired gradients (orange). Recovered packets are drawn in orange."),
                  (19.6e6, "End-to-end receipts: a source that got no receipt re-sends (orange outline). Copies that waited too long in a waiting room are dropped (red cross) and recovered this way."),
                  (23e6, "Fast forward to the end of the run.")],
            legend=[('data', 'data packet (ring = target region)'), ('recov', 'salvaged or re-sent packet'), ('rcpt', 'end-to-end receipt'),
                    ('failed', 'failed, not yet detected'), ('dead', 'failed and detected'), ('detect', 'neighbour declares it dead'),
                    ('repair', 'gradient repair messages'), ('salv', 'salvage / re-send'), ('drop', 'waiting-room timeout')],
            kind='fail', regionkey=True)
    if name == 'hotspot':
        return dict(
            runs=['hot_none', 'hot_detour'], labels=["Always the lowest-id neighbour", "RADRA load-aware forwarding"],
            mode='load', title="RADRA: load balancing under a hotspot",
            sub="Two real runs of the final RADRA code in VisibleSim. Arms and legs each stream 10 packets per module to the head (1,600 packets). Only the forwarding choice differs.",
            intro=[(FT, "RADRA in action: load balancing", INK),
                   (F, "Arms and legs stream 1,600 packets to the head. Left: each module always uses the same closer neighbour. Right: RADRA picks the least loaded closer neighbour and may take a bounded sideways detour.", GREY),
                   (FS, "Module colour = data messages relayed so far (yellow → dark red). Paths are equally short on both sides.", SOFT)],
            phases=[(4.99e6, 6.0e6, slow), (6.0e6, 9.5e6, med)],
            caps=[(0, "Both runs send exactly the same packets at the same times."),
                  (5.35e6, "Left: all packets funnel through the same modules of the neck and torso. Right: equally short alternatives are used, and some packets step sideways (magenta) around busy modules."),
                  (6.5e6, "Watch the colour: on the left two columns turn dark red; on the right the load is spread across the torso."),
                  (8.8e6, "Same delivery (1,600 / 1,600 on both sides), but the busiest module and the latency drop with load-aware forwarding.")],
            legend=[('data', 'data packet'), ('rcpt', 'end-to-end receipt'), ('detour', 'sideways detour (right only)'),
                    ('load', 'data messages relayed (0 → 700+)')],
            kind='hot')
    if name == 'torn':
        return dict(
            runs=['ring', 'ring_notorn'], labels=["RADRA (torn-region handling)", "Without torn-region handling"],
            mode='region', title="RADRA: a region torn in two",
            sub="Two real runs in VisibleSim. Ring robot, 272 modules, unicast. At t = 15 s two failures cut the yellow region into two pieces.",
            intro=[(FT, "RADRA in action: a region torn in two", INK),
                   (F, "Two failures cut one region of a ring-shaped robot into two pieces. Packets for the cut-off piece must now enter it from the other side. Left: RADRA. Right: the same run without torn-region handling and receipts.", GREY),
                   (FS, "Each dot is a real message of the run.", SOFT)],
            phases=[(14.7e6, 15.3e6, slow), (15.3e6, 16.0e6, med * 2), (16.0e6, 19.0e6, med), (19.0e6, 23.5e6, med * 2), (23.5e6, 33.2e6, ff)],
            caps=[(0, "Before the cut, packets reach the yellow region by the shortest way round the ring."),
                  (15.0e6, "t = 15 s: two modules fail. The yellow region is now in two pieces, and the short way into the left piece is gone."),
                  (16.1e6, "The failures are detected. The pieces of the region announce themselves locally, so a packet for a cell in the cut-off piece follows that piece's own gradient around the ring."),
                  (19.5e6, "Left: if a piece was not announced far enough, the search widens and receipts re-send. Right: packets stuck at the cut wait, time out and are dropped (red cross); with no receipts they are lost."),
                  (23.5e6, "Fast forward. Left: every deliverable packet arrives. Right: packets addressed to the cut-off piece keep being lost.")],
            legend=[('data', 'data packet'), ('recov', 'salvaged or re-sent packet'), ('rcpt', 'end-to-end receipt'),
                    ('dead', 'failed module'), ('deliver', 'delivered'), ('drop', 'copy dropped after waiting')],
            kind='torn', stack=True)
    raise SystemExit("unknown scenario " + name)


OUTRO = {
    'anycast': ["2,000 packets sent, 2,000 delivered.", "Median latency 0.10 s, p95 0.21 s. Path stretch 1.00: every packet followed a shortest path."],
    'unicast': ["2,000 packets sent, 2,000 delivered to their exact target module.", "Median latency 0.11 s, p95 0.30 s. Path stretch 1.00."],
    'geocast': ["80 geocasts, each delivered to every module of its target region.", "Median latency 0.10 s, p95 0.20 s. Outside the target region each geocast travelled as a single copy."],
    'failures': ["40 of 400 modules failed silently. All 40 were detected and the gradients repaired locally.",
                 "1,312 packets had their source and target alive and connected at the end: all 1,312 were delivered.",
                 "388 packets were salvaged from dead neighbours and 659 re-sent by end-to-end receipts."],
    'hotspot': ["Both runs deliver 1,600 / 1,600 packets.",
                "Busiest module: 800 messages with lowest-id forwarding, 572 with RADRA.",
                "p95 latency: 1.81 s → 0.29 s, for a path stretch of only 1.03."],
    'torn': ["RADRA: 1,340 / 1,340 deliverable packets delivered after the region was torn in two.",
             "Without torn-region handling and receipts: 1,242 / 1,340, so 98 packets lost, mostly those addressed to the cut-off piece."],
}


def render(name, out, logdir='.', cfgdir='.'):
    spec = SPEC(name)
    runs = []
    n = len(spec['runs'])
    area_w = PANEL_X - 40 - 40
    for k, rn in enumerate(spec['runs']):
        R = Robot(os.path.join(cfgdir, rn + '.xml'))
        if spec.get('stack'):
            h = (H - 120) / n
            R.place(50, 80 + k * h + 40, area_w - 20, h - 70)
        else:
            w = area_w / n
            R.place(40 + k * w + 10, 90, w - 20, H - 150)
        runs.append(Run(R, os.path.join(logdir, rn + '.csv')))
    cfg = {}
    if spec['kind'] == 'torn':
        R0 = runs[0].R
        cfg['left'] = {m for m in R0.pos if R0.reg[m] == 1 and R0.pos[m][0] < 11}
    states = [State(r) for r in runs]
    tmp = f"_frames_{name}"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    k = 0
    intro = os.path.join(tmp, "intro.png")
    card(intro, spec['intro'])
    for _ in range(int(4.5 * FPS)):
        shutil.copy(intro, f"{tmp}/f{k:05d}.png")
        k += 1
    for fi, (T, dt) in enumerate(timeline(spec['phases'])):
        im = Image.new('RGB', (W, H), 'white')
        d = ImageDraw.Draw(im)
        for j, (run, st) in enumerate(zip(runs, states)):
            st.advance(T, dt, fi, cfg)
            draw_robot(d, run.R, st, T, fi, spec['mode'])
            draw_packets(d, run.R, run, st, T, dt, spec.get('gcfill', False))
            if 'labels' in spec:
                lab = spec['labels'][j]
                tw = d.textlength(lab, font=FH)
                cx = run.R.ox + run.R.cell * run.R.GW / 2
                ly = run.R.oy - 45 if spec.get('stack') else 30
                d.text((cx - tw / 2, ly), lab, fill=INK, font=FH)
        if spec.get('regionkey'):
            x = 60
            for r in range(1, 7):
                d.rectangle([x, H - 52, x + 22, H - 30], fill=mix(RC[r], (255, 255, 255), 0.45))
                d.text((x + 30, H - 53), RNAME[r], fill=GREY, font=FXS)
                x += 40 + d.textlength(RNAME[r], font=FXS) + 30
        rows = metrics(spec, states, T)
        panel(d, spec, T, dt, rows, caption_at(spec['caps'], T), spec['legend'])
        im.save(f"{tmp}/f{k:05d}.png")
        k += 1
    last = f"{tmp}/f{k - 1:05d}.png"
    for _ in range(int(2 * FPS)):
        shutil.copy(last, f"{tmp}/f{k:05d}.png")
        k += 1
    outro = os.path.join(tmp, "outro.png")
    card(outro, [(FT, "Result of this run", INK)] + [(F, x, GREY) for x in OUTRO[name]] +
         [(FS, "Final counters as printed by the simulator (seed 1). Reproduce: configs/videos/" + spec['runs'][0] + ".xml", SOFT)])
    for _ in range(int(5 * FPS)):
        shutil.copy(outro, f"{tmp}/f{k:05d}.png")
        k += 1
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", f"{tmp}/f%05d.png",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "24", "-pix_fmt", "yuv420p", out], check=True)
    shutil.rmtree(tmp)
    return k


def metrics(spec, states, T):
    kind = spec['kind']
    if kind in ('basic', 'fail'):
        s = states[0]
        inflight = s.sent - len(s.delivered)
        rows = [("packets sent", f"{s.sent:,}"), ("packets delivered", f"{len(s.delivered):,}"),
                ("not yet delivered", f"{max(0, inflight):,}"),
                ("median / p95 latency", f"{s.pct(.5):.2f} s / {s.pct(.95):.2f} s"), (None, None),
                ("data hops", f"{s.data_hops:,}"), ("receipt hops", f"{s.rcpt_hops:,}"),
                ("hops per packet", f"{s.data_hops / max(1, s.sent):.1f}")]
        if kind == 'fail':
            dd = [s.detected[m] - s.killed[m] for m in s.detected if m in s.killed]
            rows += [(None, None), ("modules failed", f"{sum(1 for t in s.killed.values() if t <= T)}"),
                     ("failures detected", f"{len(s.detected)}"),
                     ("mean detection delay", f"{(sum(dd) / len(dd) / 1e6) if dd else 0:.2f} s"),
                     ("repair messages", f"{s.ctl - s.ctl_at_fail:,}" if s.killed else "0"),
                     ("packets salvaged", f"{s.salv:,}"), ("waiting-room timeouts", f"{s.expired:,}"),
                     ("re-sent by receipts", f"{s.resent:,}")]
        return rows
    if kind == 'hot':
        rows = []
        for lab, s in zip(["lowest-id", "RADRA"], states):
            busiest = max(s.load.values()) if s.load else 0
            rows += [(f"{lab}: delivered", f"{len(s.delivered):,} / {s.sent:,}"),
                     (f"{lab}: busiest module", f"{busiest:,} msgs"),
                     (f"{lab}: p95 latency", f"{s.pct(.95):.2f} s")]
            if lab == "RADRA":
                rows.append(("RADRA: sideways detours", f"{s.detours:,}"))
            rows.append((None, None))
        return rows[:-1]
    if kind == 'torn':
        rows = []
        for lab, s in zip(["RADRA", "without"], states):
            rows += [(f"{lab}: delivered", f"{len(s.delivered):,} / {s.sent:,}"),
                     (f"{lab}: into the cut piece", f"{s.lefthits:,}"),
                     (f"{lab}: copies dropped", f"{s.drops:,}"),
                     (f"{lab}: salvaged / re-sent", f"{s.salv:,} / {s.resent:,}")]
            rows.append((None, None))
        return rows[:-1]


if __name__ == "__main__":
    a = sys.argv
    render(a[1], a[2], a[3] if len(a) > 3 else '.', a[4] if len(a) > 4 else '.')
