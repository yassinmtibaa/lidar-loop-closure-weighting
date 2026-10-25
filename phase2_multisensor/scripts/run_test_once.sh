#!/usr/bin/env bash
# P2.1d test runs: default x3 and W=1e3 x3 on the 3 test sessions, run once.
cd /home/yassine/lidar_loop/phase2_multisensor
export SEQS="2012-02-12 2012-06-15 2013-04-05"
for r in 1 2 3; do
  echo "round $r start $(date -Is)"
  ( unset KS_ODOM_INFO KS_CLOSURE_INFO; ./scripts/run_slam.sh slam_default_r$r configs/kiss_slam_default.yaml 3 ) &
  ( KS_ODOM_INFO=1,1,1,1e3,1e3,1e3 KS_CLOSURE_INFO=1,1,1,1e3,1e3,1e3 ./scripts/run_slam.sh w_rot1e3_r$r configs/kiss_slam_default.yaml 3 ) &
  wait
  echo "round $r end $(date -Is)"
done
echo ALL_DONE
