#!/bin/bash
# Event logs behind Fig. 3 and the videos (three runs of the final version, a few minutes).
# Usage: tools/make_logs.sh <VisibleSim dir>     (run from the repository root, after extracting the configs)
set -e; VS=${1:-$HOME/VisibleSim}; B=$VS/applicationsBin/bbRDRM5/bbRDRM5; mkdir -p logs
RR_LOG=logs/r10.csv        $B -c configs/A__plain__anycast__r10s1__final.xml -t -a 1 | grep RESULT2
RR_LOG=logs/hot_none.csv   $B -c configs/F_L__plain__hotspot__none__final__none.xml -t -a 1 | grep RESULT3
RR_LOG=logs/hot_detour.csv $B -c configs/F_L__plain__hotspot__none__final__detour.xml -t -a 1 | grep RESULT3
