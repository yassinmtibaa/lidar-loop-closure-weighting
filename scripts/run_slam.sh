#!/usr/bin/env bash
# usage: [SEQS="00 01 ..."] ./scripts/run_slam.sh <name> <config.yaml> [parallel_jobs]
NAME=$1; CFG=$(realpath "$2"); JOBS=${3:-3}
SEQS=${SEQS:-"00 01 02 03 04 05 06 07 08 09 10"}
export NAME CFG
printf "%s\n" $SEQS | xargs -P "$JOBS" -I{} bash -c '
  OUT=$HOME/lidar_loop/results/${NAME}_seq{}
  TMP=/tmp/slam_${NAME}_{}.yaml
  sed "s|^out_dir:.*|out_dir: $OUT|" "$CFG" > "$TMP"
  kiss_slam_pipeline --dataloader kitti --sequence {} --config "$TMP" \
    $HOME/lidar_loop/data/kitti/dataset > $HOME/lidar_loop/results/${NAME}_seq{}.log 2>&1
  rm -f "$TMP"
'
