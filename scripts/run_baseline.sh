#!/usr/bin/env bash
# usage: ./scripts/run_baseline.sh <experiment_name>
set -e
NAME=${1:-baseline}
for s in 00 01 02 03 04 05 06 07 08 09 10; do
  export kiss_icp_out_dir=$HOME/lidar_loop/results/${NAME}_seq${s}
  kiss_icp_pipeline --dataloader kitti --sequence $s $HOME/lidar_loop/data/kitti/dataset
done
