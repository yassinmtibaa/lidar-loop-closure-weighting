#!/usr/bin/env bash
# usage: [SEQS="2012-01-15 ..."] [KS_ODOM_INFO=...] [KS_CLOSURE_INFO=...] ./scripts/run_slam.sh <name> <config.yaml> [parallel_jobs]
cd "$(dirname "$0")/.."
NAME=$1; CFG=$(realpath "$2"); JOBS=${3:-3}; ROOT=$PWD
SEQS=${SEQS:-"2012-01-15 2012-04-29 2012-08-04"}
export NAME CFG ROOT
printf "%s\n" $SEQS | xargs -P "$JOBS" -I{} bash -c '
  OUT=$ROOT/results/${NAME}_seq{}
  TMP=$(mktemp /tmp/slam_${NAME}_{}_XXXX.yaml)
  sed "s|^out_dir:.*|out_dir: $OUT|" "$CFG" > "$TMP"
  kiss_slam_pipeline --dataloader nclt --config "$TMP" $ROOT/data/nclt/{} > $ROOT/results/${NAME}_seq{}.log 2>&1
  rm -f "$TMP"
  echo "{} done"
'
