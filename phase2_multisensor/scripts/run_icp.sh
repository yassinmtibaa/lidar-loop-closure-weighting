#!/usr/bin/env bash
# usage: [SEQS="2012-01-15 ..."] ./scripts/run_icp.sh <name> [parallel_jobs]   (KISS-ICP defaults, no config)
cd "$(dirname "$0")/.."
NAME=$1; JOBS=${2:-3}; ROOT=$PWD
SEQS=${SEQS:-"2012-01-15 2012-04-29 2012-08-04"}
export NAME ROOT
printf "%s\n" $SEQS | xargs -P "$JOBS" -I{} bash -c '
  export kiss_icp_out_dir=$ROOT/results/${NAME}_seq{}
  kiss_icp_pipeline --dataloader nclt $ROOT/data/nclt/{} > $ROOT/results/${NAME}_seq{}.log 2>&1
  echo "{} done: $(tail -1 $ROOT/results/${NAME}_seq{}.log | cut -c1-60)"
'
