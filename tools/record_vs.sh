#!/bin/bash
# Screen-record a VisibleSim run (GUI) on a virtual display, cropped to the robot.
# Usage: tools/record_vs.sh ~/VisibleSim <config.xml> <out.mp4> [seconds=24]
# Needs: Xvfb, ffmpeg, python3 with Pillow and numpy.
VS=$1; CFG=$(realpath "$2"); OUT=$(realpath -m "$3"); SECS=${4:-24}
B=$VS/applicationsBin/bbRDRM5/bbRDRM5
Xvfb :99 -screen 0 1600x1000x24 >/dev/null 2>&1 & XP=$!; sleep 2
(cd "$(dirname "$B")" && DISPLAY=:99 timeout $((SECS+15)) ./bbRDRM5 -c "$CFG" -r >/dev/null 2>&1) & SP=$!
sleep 2.5
DISPLAY=:99 ffmpeg -loglevel error -y -f x11grab -video_size 1600x1000 -framerate 20 -i :99 -t "$SECS" \
  -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p /tmp/raw_rec.mp4
kill $SP $XP 2>/dev/null; pkill -f "bbRDRM5 -c $CFG" 2>/dev/null
ffmpeg -loglevel error -y -sseof -1 -i /tmp/raw_rec.mp4 -frames:v 1 /tmp/last.png
read X Y W H < <(python3 - <<'P'
from PIL import Image; import numpy as np
a=np.asarray(Image.open('/tmp/last.png').convert('RGB')).astype(int)
ys,xs=np.where((a.max(2)-a.min(2))>40); pad=40
x0=max(0,xs.min()-pad); x1=min(a.shape[1],xs.max()+pad); y0=max(0,ys.min()-pad); y1=min(a.shape[0],ys.max()+pad)
print(x0,y0,(x1-x0)//2*2,(y1-y0)//2*2)
P
)
ffmpeg -loglevel error -y -i /tmp/raw_rec.mp4 -vf "crop=$W:$H:$X:$Y" -c:v libx264 -preset veryfast -crf 21 -pix_fmt yuv420p "$OUT"
echo "wrote $OUT"
