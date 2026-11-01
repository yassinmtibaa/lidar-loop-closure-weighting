#!/usr/bin/env python3
"""Judge the tune-session sweep (DECISIONS 2026-10-27, T1-T7). LiDAR subsets only until the supercomputer run: T1, T2
are verdicts; T3, T4, T6, T7 are partial readings over the subsets present. Writes results/tune_verdicts.txt."""
import csv
import glob
import os
from collections import defaultdict

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TUNE = ["2012-01-15", "2012-04-29", "2012-08-04"]
ORDER = "LIWFG"
key = lambda s: "".join(c for c in ORDER if c in s)


def load(seq, w):
    suffix = "" if w == "predicted" else f"_{w}"
    p = [f for f in glob.glob(os.path.join(ROOT, "results", f"subsets_{seq}_noise_v5_*{suffix}.csv"))
         if w != "predicted" or not f.endswith(("_fixed.csv", "_identity.csv"))]
    assert len(p) == 1, p
    d = defaultdict(list)
    for r in csv.DictReader(open(p[0])):
        d[r["subset"]].append(r)
    return d


def verdict(a, b):
    """a -> b: 3-run ATE ranges must not overlap and the mean change must exceed 5%."""
    A, B = [float(r["ate"]) for r in a], [float(r["ate"]) for r in b]
    if not all(r["converged"] == "True" for r in a + b):
        return "undecided"
    if min(B) > max(A) and np.mean(B) > 1.05 * np.mean(A):
        return "worse"
    if max(B) < min(A) and np.mean(B) < 0.95 * np.mean(A):
        return "better"
    return "within"


out = []
tally = {}
for seq in TUNE:
    for w in ("predicted", "fixed", "identity"):
        d = load(seq, w)
        c = defaultdict(int)
        worse = []
        for s in d:
            for x in ORDER:
                t = key(s + x)
                if x not in s and t in d:
                    v = verdict(d[s], d[t])
                    c[v] += 1
                    if v == "worse":
                        worse.append(f"{s}+{x}")
        conv = sum(all(r["converged"] == "True" for r in d[s]) for s in d)
        nees = [np.mean([float(r["nees_mean"]) for r in d[s]]) for s in d if all(r["converged"] == "True" for r in d[s])]
        inb = sum(1 <= n <= 9 for n in nees)
        tally[seq, w] = (c, conv, len(d), inb, len(nees), d)
        dec = c["better"] + c["within"] + c["worse"]
        out.append(f"{seq} {w:9s}: converged {conv}/{len(d)}; additions better {c['better']}, within {c['within']}, worse "
                   f"{c['worse']} ({100 * c['worse'] / max(dec, 1):.0f}% of {dec} decided), undecided {c['undecided']}; "
                   f"NEES in [1,9] {inb}/{len(nees)}")
        out.append(f"   worse: {' '.join(worse)}")
out.append("")
out.append("ATE mean (m) / NEES mean, predicted | fixed | identity")
for s in ["L", "LI", "LW", "LF", "LG", "LIF", "LIG", "LIWF", "LIWFG"]:
    for seq in TUNE:
        vals = []
        for w in ("predicted", "fixed", "identity"):
            d = tally[seq, w][5][s]
            vals.append(f"{np.mean([float(r['ate']) for r in d]):8.2f} /{np.mean([float(r['nees_mean']) for r in d]):9.2f}"
                        + ("" if all(r["converged"] == "True" for r in d) else "*"))
        out.append(f"{s:6s} {seq}  " + " | ".join(vals))
out.append("* = not all runs converged")
out.append("")
p = {seq: tally[seq, "predicted"] for seq in TUNE}
t1 = [verdict(p[s][5]["L"], p[s][5]["LI"]) for s in TUNE]
t2 = [verdict(p[s][5]["L"], p[s][5]["LG"]) for s in TUNE]
out.append(f"T1 (L -> L+I better on >= 2 of 3): {t1} -> {'MET' if t1.count('better') >= 2 else 'NOT MET'}")
out.append(f"T2 (L -> L+G worse on >= 2 of 3): {t2} -> {'MET' if t2.count('worse') >= 2 else 'NOT MET'}")
pct = [p[s][0]["worse"] / max(sum(p[s][0][k] for k in ("better", "within", "worse")), 1) for s in TUNE]
out.append(f"T3 partial (worse < 25% on every session): {[f'{x:.0%}' for x in pct]}")
out.append(f"T4 partial (converged, L subsets only): {[f'{p[s][1]}/{p[s][2]}' for s in TUNE]}")
out.append("T5: NOT MET as worded, see DECISIONS 2026-10-31")
out.append("T6 partial (identity worse count > predicted, every session): "
           + str([(tally[s, 'identity'][0]['worse'], p[s][0]['worse']) for s in TUNE]))
out.append(f"T7 partial (under half of converged subsets with NEES in [1,9]): {[f'{p[s][3]}/{p[s][4]}' for s in TUNE]}")
text = "\n".join(out)
print(text)
open(os.path.join(ROOT, "results", "tune_verdicts.txt"), "w").write(text + "\n")
