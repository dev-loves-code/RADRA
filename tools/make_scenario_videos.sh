#!/bin/bash
# Run the six video scenarios with the event log on, then render them.
# Usage: bash tools/make_scenario_videos.sh ~/VisibleSim   (from the repository root)
VS=$1; ROOT=$(pwd); OUT=$ROOT/videos/scenarios; W=$(mktemp -d)
cp configs/videos/*.xml $W/
for s in anycast unicast geocast failures hot_none hot_detour ring ring_notorn; do
  (cd $VS/applicationsBin/bbRDRM5 && RR_LOG=$W/$s.csv ./bbRDRM5 -c $W/$s.xml -t -a 1 > /dev/null) &
done; wait
mkdir -p $OUT; cd $W
for v in anycast unicast geocast failures hotspot torn; do python3 $ROOT/tools/scenario_video.py $v $OUT/$v.mp4 . . ; done
echo "videos in $OUT"
