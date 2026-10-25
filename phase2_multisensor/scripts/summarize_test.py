#!/usr/bin/env python3
"""P2.1d test summary. Reads results/{slam_default,w_rot1e3}_r{1,2,3}_seq<test>/latest, scores each run with
metrics.py, prints per-session ranges/means, the non-overlap verdicts and the P2.1d verdict (rule fixed 2026-10-10/14:
W lowers the KITTI metric vs default on average over the test sessions (3-run means), without raising the 3-run mean
ATE by more than 10% on any session). Mechanical: the text in DECISIONS.md is written by a person/Claude from this."""
import os, re, subprocess, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQS = ["2012-02-12", "2012-06-15", "2013-04-05"]
CONDS = {"default": "slam_default", "W=1e3": "w_rot1e3"}
res = {}
for c, pre in CONDS.items():
    for s in SEQS:
        for r in (1, 2, 3):
            d = f"{ROOT}/results/{pre}_r{r}_seq{s}/latest"
            log = f"{ROOT}/results/{pre}_r{r}_seq{s}.log"
            if not os.path.exists(f"{d}/{s}_poses_kitti.txt"):
                print(f"MISSING {pre}_r{r} {s}"); continue
            out = subprocess.run([sys.executable, f"{ROOT}/scripts/metrics.py", d, s], capture_output=True, text=True).stdout.strip()
            _, ate, rpe, kt, dr = out.split(",")
            cl = len(re.findall("Closure Accepted", open(log).read()))
            res.setdefault((c, s), []).append((float(ate), float(kt), cl))
            print(f"run {c:8s} r{r} {s}: ATE {float(ate):.3f} m  KITTI {float(kt):.3f}%  closures {cl}  {dr}")
mean = lambda v: sum(v) / len(v)
print("\nsession | metric | default range (mean) | W range (mean) | non-overlap verdict")
kd, kw, ate_ok = [], [], True
for s in SEQS:
    if ("default", s) not in res or ("W=1e3", s) not in res: continue
    for i, m in ((0, "ATE m"), (1, "KITTI %")):
        a = [x[i] for x in res[("default", s)]]; b = [x[i] for x in res[("W=1e3", s)]]
        v = "W BETTER" if max(b) < min(a) else "W WORSE" if min(b) > max(a) else "within noise"
        print(f"{s} | {m} | {min(a):.3f}-{max(a):.3f} ({mean(a):.3f}) | {min(b):.3f}-{max(b):.3f} ({mean(b):.3f}) | {v}")
        if i == 1: kd.append(mean(a)); kw.append(mean(b))
        else:
            ch = 100 * (mean(b) / mean(a) - 1); print(f"   ATE mean change W vs default: {ch:+.1f}%")
            ate_ok &= ch <= 10
n = sum(len(v) for v in res.values())
print(f"\nruns scored: {n}/18")
if kd:
    print(f"KITTI mean over sessions: default {mean(kd):.3f}%, W {mean(kw):.3f}% ({100*(mean(kw)/mean(kd)-1):+.1f}%)")
    print(f"P2.1d: KITTI lower on average: {mean(kw) < mean(kd)}; no session ATE mean +>10%: {ate_ok}; "
          f"=> {'MET' if mean(kw) < mean(kd) and ate_ok and n == 18 else 'NOT MET' if n == 18 else 'INCOMPLETE'}")
