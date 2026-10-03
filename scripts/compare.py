#!/usr/bin/env python3
"""usage: compare.py baseline.csv exp1.csv [exp2.csv ...]
Percent change vs baseline on shared sequences (negative = better)."""
import csv, sys

def load(p):
    return {r["seq"]: (float(r["ate_rmse"]), float(r["rpe100_rmse"]))
            for r in csv.DictReader(open(p))}

base = load(sys.argv[1])
for p in sys.argv[2:]:
    ex = load(p)
    seqs = sorted(set(base) & set(ex))
    print(f"\n{p}\nseq    ATE%    RPE%")
    ate_d, rpe_d = [], []
    for s in seqs:
        da = 100 * (ex[s][0] / base[s][0] - 1)
        dr = 100 * (ex[s][1] / base[s][1] - 1)
        ate_d.append(da); rpe_d.append(dr)
        print(f"{s}  {da:+7.1f} {dr:+7.1f}")
    print(f"mean {sum(ate_d)/len(ate_d):+7.1f} {sum(rpe_d)/len(rpe_d):+7.1f}")
