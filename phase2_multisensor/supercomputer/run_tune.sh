#!/usr/bin/env bash
# Phase 2.4 sweep on the TUNE sessions (protocol: DECISIONS 2026-10-27 "2.4 tune-session protocol").
# Never point this at validation or test sessions. Run from phase2_multisensor/. JOBS = cores per session.
#   step 1  KISS-ICP x3 per session            (~15 min per run for 2012-01-15 at ~37 frames/s)
#   step 2  fit noise model v5 per session     (minutes)
#   step 3  31 subsets x 3 inputs x 3 weightings (predicted, fixed, identity) per session, with position NEES;
#           noise fitted on the OTHER two tune sessions (leave-one-out)
set -euo pipefail
cd "$(dirname "$0")/.."
JOBS=${JOBS:-32}
ONLY=${ONLY:-}          # optional comma-separated subset list (local partial runs)
TUNE=(2012-01-15 2012-04-29 2012-08-04)
PY=.venv/bin/python
step=${1:-all}

if [[ $step == all || $step == icp ]]; then
  for s in "${TUNE[@]}"; do for r in 1 2 3; do
    out=results/icp_tune_r${r}_seq$s
    [[ -f $out/latest/${s}_poses.npy ]] && continue
    kiss_icp_out_dir=$PWD/$out kiss_icp_pipeline --dataloader nclt data/nclt/$s > $out.log 2>&1 &
  done; done; wait
  for s in "${TUNE[@]}"; do for r in 1 2 3; do $PY scripts/metrics.py results/icp_tune_r${r}_seq$s/latest $s; done; done \
    | tee results/icp_tune_metrics.txt
fi

if [[ $step == all || $step == noise ]]; then
  for s in "${TUNE[@]}"; do
    $PY scripts/subsets_run.py --seq $s --runs icp_tune_r1,icp_tune_r2,icp_tune_r3 \
      --fit-noise results/noise_v5_$s.json > results/noise_v5_$s.txt
  done
fi

if [[ $step == all || $step == sweep ]]; then
  for i in 0 1 2; do
    s=${TUNE[$i]}
    others=$(for j in 0 1 2; do if [[ $j != $i ]]; then printf "results/noise_v5_%s.json," "${TUNE[$j]}"; fi; done)
    for w in predicted fixed identity; do
      $PY scripts/subsets_run.py --seq $s --runs icp_tune_r1,icp_tune_r2,icp_tune_r3 \
        --noise "${others%,}" --weights $w --nees 50 --jobs "$JOBS" ${ONLY:+--only "$ONLY"}
    done
  done
fi
