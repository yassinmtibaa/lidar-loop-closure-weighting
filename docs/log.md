# Lab log

## 2026-10-03
**Goal:** KITTI baseline with KISS-ICP.
**Did:** fixed ROS/NVIDIA setup earlier; built KISS-ICP; patched TUM export bug;
ran sequences 00-10; evaluated with evo.
**Result:** ATE 0.24-6.46 m, translation error 0.28-0.82% (results/baseline.csv).
**Surprises:** seq 04 rotational error (0.623 rad) looks like an alignment
artifact on a short straight track; seq 08 has heavy-tailed RPE.
**Next:** inspect configs/baseline.yml, design first preprocessing sweep.

## 2026-10-03 (night)
**Goal:** KISS-SLAM defaults on all 11 KITTI sequences.
**Result:** ATE -74% (00), -71% (02), -56% (05), -22% (06); no gain on 07 (1 closure)
and 09 (0 closures); 08 worse (+11%, 5 closures). RPE +39% on 05, mean +5.1%.
**Surprises:** closure-free sequences differ slightly from plain KISS-ICP (+/-1-4%).
**Next:** inspect 05 plots, then decide whether a learned component has a target.

## 2026-10-03 (late)
**Goal:** diagnose RPE regression of KISS-SLAM vs odometry.
**Result:** scripts/rpe_segments.py (per-100m RPE, self-check matches evo). On 05, SLAM RPE
rmse 0.69 vs 0.48; seg 10 (1.89 m) is half the gap; segs 4,7,9,10 near closure vertices.
On 00/02 the optimizer changes segment motion by ~0.10-0.13 m rms (~11% of baseline RPE)
and net RPE is only +2.7/+3.3%. Mean RPE change excluding 05 is ~+1.7%.
**Conclusion so far:** regression mostly a 05 (and 06) issue; correction-jump hypothesis
unproven. Need 05 diff table and closure edges.
