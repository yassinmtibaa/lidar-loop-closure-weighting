#!/usr/bin/env python3
"""Judge P2.3a-d (DECISIONS 2026-10-25) from results/subsets_smoke_2013-01-10.csv.
A pair S -> S+x is judged only if every run of both subsets converged; otherwise it is 'undecided'."""
import csv
import os
from collections import defaultdict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
rows = list(csv.DictReader(open(os.path.join(ROOT, "results", "subsets_smoke_2013-01-10.csv"))))
ate, conv, fin = defaultdict(list), defaultdict(lambda: True), defaultdict(lambda: True)
for r in rows:
    ate[r["subset"]].append(float(r["ate"]))
    conv[r["subset"]] &= r["converged"] == "True"
    fin[r["subset"]] &= r["finite"] == "True"
ORDER = "LIWFG"
key = lambda s: "".join(c for c in ORDER if c in s)
out = []
out.append(f"P2.3a finite for all 31 x 3: {all(fin.values()) and len(fin) == 31}; "
           f"converged (< 500 solver iterations): {sum(conv.values())}/31 -> "
           f"not converged: {' '.join(sorted((s for s in conv if not conv[s]), key=lambda s: (len(s), s)))}")
path = 1138.8
for s in ("I", "F", "IF"):
    out.append(f"P2.3b {s}: ATE {min(ate[s]):.1f} m (> {0.05 * path:.0f} m: {min(ate[s]) > 0.05 * path}), "
               f"converged {conv[s]}")
counts = defaultdict(lambda: defaultdict(int))
lines = []
for s in ate:
    for x in ORDER:
        if x in s:
            continue
        t = key(s + x)
        a, b = ate[s], ate[t]
        if not (conv[s] and conv[t]):
            v = "undecided"
        else:
            ma, mb = sum(a) / len(a), sum(b) / len(b)
            if min(b) > max(a) and mb > 1.05 * ma:
                v = "worse"
            elif max(b) < min(a) and mb < 0.95 * ma:
                v = "better"
            else:
                v = "within"
        counts[x][v] += 1
        counts["all"][v] += 1
        lines.append(f"{s:5s} + {x} -> {t:5s}: ATE {min(a):8.3f}-{max(a):8.3f} -> {min(b):8.3f}-{max(b):8.3f}  {v}")
for x in list(ORDER) + ["all"]:
    out.append(f"adding {x:3s}: " + ", ".join(f"{k} {counts[x][k]}" for k in ("better", "within", "worse", "undecided")))
g = counts["G"]
out.append(f"P2.3c (no addition of G is worse; 15 pairs): worse {g['worse']}, undecided {g['undecided']} -> "
           + ("NOT MET" if g["worse"] else ("MET on the decided pairs" if g["undecided"] else "MET")))
out.append(f"P2.3d (at least one addition worse; 75 pairs): worse {counts['all']['worse']} -> "
           + ("MET" if counts["all"]["worse"] else "NOT MET"))
text = "\n".join(out + [""] + sorted(lines, key=lambda l: (l.split()[2], len(l.split()[0]), l)))
print(text)
open(os.path.join(ROOT, "results", "subsets_verdicts_2013-01-10.txt"), "w").write(text + "\n")
