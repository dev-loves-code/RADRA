#!/usr/bin/env python3
"""
Generate humanoid-shaped Blinky Blocks configurations with 6 regions.

The robot stands in the x-z plane (y = 0), z is up, seen from the front.
"Left" = smaller x (the viewer's left).  Scale factor k multiplies every
dimension, so the module count grows as ~k^2 (k=1: 100 modules).

  R1 head        R2 body (torso + neck)
  R3 left arm    R4 right arm  (shoulder bar + hanging arm, 1 empty column from the torso)
  R5 left leg    R6 right leg  (2k-wide gap between the legs)

Each region gets one BFS representative ("root"): the module closest to the
region's centroid (ties -> smallest x, then z).  Sources for the traffic are
either every module ("all") or a sample of n per region ("sample").

Usage: make_humanoid.py k name out.xml [all|sample:n[:seed]|trace:srcRegion:dstRegion] [spacing_us] [failure]

spacing_us = 0 (default): every source sends all its packets at once (heavy load).
spacing_us > 0: packets leave one at a time, spacing_us apart (light load, no overlap).

failure (modules cut out at t = 1.5 s, after setup and before traffic):
  shoulder        the torso module at the left shoulder joint (the busiest BFS module)
  bodyroot        the BFS representative of the body (R2)
  armcut          the whole left-shoulder joint column: the left arm is cut off the robot
  random:pct:seed pct % of all modules, chosen at random
  cells:x,z;x,z   an explicit list of modules

trace modes: trace:srcRegion:dstRegion (source = farthest module of srcRegion)
             tracepos:x:z:dstRegion   (source = the module at x, z)
"""
import sys, random

COLORS = {1: "240,200,60",   # head  - yellow
          2: "70,130,220",   # body  - blue
          3: "60,180,90",    # left arm  - green
          4: "40,150,150",   # right arm - teal
          5: "220,90,70",    # left leg  - red
          6: "170,80,190"}   # right leg - purple
NAMES = {1: "head", 2: "body", 3: "left arm", 4: "right arm", 5: "left leg", 6: "right leg"}

def humanoid(k):
    cells = {}
    def box(x0, x1, z0, z1, r):            # inclusive-exclusive ranges
        for x in range(x0, x1):
            for z in range(z0, z1):
                assert (x, z) not in cells, (x, z)
                cells[(x, z)] = r
    # legs
    box(2*k, 4*k, 0, 6*k, 5)
    box(6*k, 8*k, 0, 6*k, 6)
    # torso + neck
    box(2*k, 8*k, 6*k, 13*k, 2)
    box(4*k, 6*k, 13*k, 14*k, 2)
    # head
    box(3*k, 7*k, 14*k, 18*k, 1)
    # arms: shoulder bar (top k rows of torso height) + hanging arm
    box(0, 2*k, 12*k, 13*k, 3)              # left shoulder bar (touches torso at x=2k-1|2k)
    box(0, k, 6*k, 12*k, 3)                 # left arm, column gap x in [k,2k)
    box(8*k, 10*k, 12*k, 13*k, 4)
    box(9*k, 10*k, 6*k, 12*k, 4)
    return cells, 10*k, 18*k

def roots(cells):
    out = {}
    for r in range(1, 7):
        pts = [p for p, rr in cells.items() if rr == r]
        cx = sum(p[0] for p in pts) / len(pts); cz = sum(p[1] for p in pts) / len(pts)
        out[r] = min(pts, key=lambda p: ((p[0]-cx)**2 + (p[1]-cz)**2, p[0], p[1]))
    return out

def main():
    k = int(sys.argv[1]); name = sys.argv[2]; out = sys.argv[3]
    mode = sys.argv[4] if len(sys.argv) > 4 else "all"
    hop_delay = 150000 if mode.startswith("trace") else 0
    spacing = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    failure = sys.argv[6] if len(sys.argv) > 6 else ""
    cells, W, H = humanoid(k)
    rt = roots(cells)
    order = sorted(cells, key=lambda p: (p[1], p[0]))      # ids ORDERED: bottom row first
    src = set(); exp_sources = "all"; trace = ""
    if mode.startswith("sample"):
        parts = mode.split(":"); n = int(parts[1]); seed = int(parts[2]) if len(parts) > 2 else 1
        rnd = random.Random(seed); exp_sources = "marked"
        for r in range(1, 7):
            pts = sorted(p for p in order if cells[p] == r)
            src |= set(rnd.sample(pts, min(n, len(pts))))
    elif mode.startswith("tracepos"):
        # one packet from the module at (x, z) to region dst
        _, x, z, dr = mode.split(":"); exp_sources = "marked"
        assert (int(x), int(z)) in cells, (x, z)
        src = {(int(x), int(z))}
        trace = f' traceDst="{int(dr)}"'
    elif mode.startswith("trace"):
        _, sr, dr = mode.split(":"); sr, dr = int(sr), int(dr); exp_sources = "marked"
        # source = the region module farthest from the destination root (a "hand" or "foot")
        tx, tz = rt[dr]
        pts = [p for p in order if cells[p] == sr]
        src = {max(pts, key=lambda p: (abs(p[0]-tx) + abs(p[1]-tz), -p[0], -p[1]))}
        trace = f' traceDst="{dr}"'
    fail = set()
    if failure == "shoulder":
        fail = {(2*k, 12*k)}
    elif failure == "bodyroot":
        fail = {rt[2]}
    elif failure == "armcut":
        fail = {(2*k, z) for z in range(12*k, 13*k)}
    elif failure.startswith("cells:"):
        fail = {tuple(int(v) for v in c.split(",")) for c in failure[6:].split(";")}
    elif failure.startswith("random"):
        _, pct, seed = failure.split(":")
        rnd = random.Random(int(seed))
        fail = set(rnd.sample(order, round(len(order) * float(pct) / 100)))
    for p in fail: assert p in cells, p
    fail_attr = ' failAt="1500000"' if fail else ""
    dmax = W + H                      # RDRM: distances >= dmax mean "unreachable"
    bs = 40
    cx, cz = W * bs / 2, H * bs / 2
    dist = int(max(W, H) * bs * 1.45)
    lines = []
    lines.append('<?xml version="1.0" standalone="no" ?>')
    lines.append(f'<!-- Humanoid, scale k={k}: {len(cells)} modules, 6 regions. Generated by make_humanoid.py -->')
    for r in range(1, 7):
        n = sum(1 for p in cells if cells[p] == r)
        lines.append(f'<!--   R{r} {NAMES[r]:9s}: {n:4d} modules, BFS root at x={rt[r][0]} z={rt[r][1]} -->')
    lines.append('<vs>')
    lines.append('  <visuals>')
    lines.append('    <window size="1600x1000" backgroundColor="0xffffff" gradientColor="0xe8e8f0"/>')
    lines.append('    <render shadows="false" grid="false"/>')
    lines.append('  </visuals>')
    lines.append(f'  <experiment name="{name}" start="3000000" sources="{exp_sources}" spacing="{spacing}" hopDelay="{hop_delay}" dmax="{dmax}"{fail_attr}{trace}/>')
    lines.append(f'  <world gridSize="{W},1,{H}">')
    lines.append(f'    <camera target="{cx:.0f},20,{cz:.0f}" thetaPhiDist="0,0,{dist}" fov="50" near="10" far="{dist*3}"/>')
    lines.append(f'    <spotlight target="{cx:.0f},20,{cz:.0f}" thetaPhiDist="-70,20,{dist*2}" fov="40" show="0" near="10" far="{dist*4}"/>')
    lines.append('    <blockList color="200,200,200">')
    for p in order:
        r = cells[p]
        extra = ""
        if rt[r] == p: extra += ' root="true"'
        if p in src: extra += ' source="true"'
        if p in fail: extra += ' fail="true"'
        lines.append(f'      <block position="{p[0]},0,{p[1]}" region="{r}" color="{COLORS[r]}"{extra}/>')
    lines.append('    </blockList>')
    lines.append('  </world>')
    lines.append('</vs>')
    open(out, "w").write("\n".join(lines) + "\n")
    print(f"{out}: k={k} N={len(cells)} grid={W}x1x{H} sources={'all' if exp_sources=='all' else len(src)}")

if __name__ == "__main__":
    main()
