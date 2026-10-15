#!/usr/bin/env bash
# usage: [SEQS="..."] ./scripts/eval.sh <name>   -> CSV: seq,ate_rmse,rpe100_rmse,kitti_trans_pct,closures
cd "$(dirname "$0")/.."
NAME=$1
SEQS=${SEQS:-"2012-01-15 2012-04-29 2012-08-04"}
echo "seq,ate_rmse,rpe100_rmse,kitti_trans_pct,closures"
for s in $SEQS; do
  d=results/${NAME}_seq$s/latest
  if [ ! -f $d/${s}_poses_kitti.txt ]; then echo "$s,MISSING,,,"; continue; fi
  ate=$(evo_ape kitti $d/${s}_gt_kitti.txt $d/${s}_poses_kitti.txt -a 2>/dev/null | awk '/rmse/{print $2}')
  rpe=$(evo_rpe kitti $d/${s}_gt_kitti.txt $d/${s}_poses_kitti.txt -d 100 -u m 2>/dev/null | awk '/rmse/{print $2}')
  kt=$(awk -F'|' '/Average Translation Error/{gsub(/ /,"",$3); print $3}' $d/result_metrics.log)
  cl=$(awk -F'|' '/closures/{gsub(/ /,"",$3); print $3}' $d/result_metrics.log)
  echo "$s,$ate,$rpe,$kt,${cl:-NA}"
done
