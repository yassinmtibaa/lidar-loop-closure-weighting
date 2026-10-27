#!/usr/bin/env bash
# Local dry run of the kit on ONE tune session (2012-04-29): checks the pipeline end to end and measures runtimes.
# Not a result: T1-T7 are judged only on the full leave-one-out sweep.
set -euo pipefail
cd "$(dirname "$0")/.."
s=2012-04-29; PY=.venv/bin/python
date +"%F %T start"
for r in 1 2 3; do
  out=results/icp_tune_r${r}_seq$s
  [[ -f $out/latest/${s}_poses.npy ]] || kiss_icp_out_dir=$PWD/$out kiss_icp_pipeline --dataloader nclt data/nclt/$s > $out.log 2>&1 &
done; wait
date +"%F %T icp done"
for r in 1 2 3; do $PY scripts/metrics.py results/icp_tune_r${r}_seq$s/latest $s; done
$PY scripts/subsets_run.py --seq $s --runs icp_tune_r1,icp_tune_r2,icp_tune_r3 --fit-noise results/dryrun_noise_v5_$s.json
date +"%F %T noise done"
for w in predicted fixed identity; do
  $PY scripts/subsets_run.py --seq $s --runs icp_tune_r1,icp_tune_r2,icp_tune_r3 --noise results/dryrun_noise_v5_$s.json \
    --weights $w --nees 50 --jobs 8 --only L,LI,LW,LF,LG,LIF,LIWFG
  date +"%F %T sweep $w done"
done
