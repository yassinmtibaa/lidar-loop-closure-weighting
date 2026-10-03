#!/usr/bin/env bash
# usage: ./scripts/run_exp.sh <name> <config.yml> [parallel_jobs]
NAME=$1; CFG=$(realpath "$2"); JOBS=${3:-3}
export NAME CFG
printf "%s\n" 00 01 02 03 04 05 06 07 08 09 10 | xargs -P "$JOBS" -I{} bash -c '
  export kiss_icp_out_dir=$HOME/lidar_loop/results/${NAME}_seq{}
  kiss_icp_pipeline --dataloader kitti --sequence {} --config "$CFG" \
    $HOME/lidar_loop/data/kitti/dataset > $HOME/lidar_loop/results/${NAME}_seq{}.log 2>&1
'
