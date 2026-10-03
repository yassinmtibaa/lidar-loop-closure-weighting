#!/usr/bin/env bash
# usage: ./scripts/eval_exp.sh <name>   (prints CSV: seq, ATE rmse, RPE-100m rmse)
NAME=$1
echo "seq,ate_rmse,rpe100_rmse"
for s in 00 01 02 03 04 05 06 07 08 09 10; do
  d=$HOME/lidar_loop/results/${NAME}_seq$s/latest
  ate=$(evo_ape kitti $d/${s}_gt_kitti.txt $d/${s}_poses_kitti.txt -a | awk '/rmse/{print $2}')
  rpe=$(evo_rpe kitti $d/${s}_gt_kitti.txt $d/${s}_poses_kitti.txt -d 100 -u m | awk '/rmse/{print $2}')
  echo "$s,$ate,$rpe"
done
