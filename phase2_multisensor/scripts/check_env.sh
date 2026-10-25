#!/usr/bin/env bash
# Checks that the research toolchain still works after a system update.
# Usage: ./check_env.sh            (checks only, ~10 s)
#        ./check_env.sh --smoke    (also runs 200 KISS-ICP frames on the smoke session)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ok(){ echo "  [OK]   $*"; }; bad(){ echo "  [FAIL] $*"; FAILS=$((FAILS+1)); }; info(){ echo "  [info] $*"; }
FAILS=0
echo "== System"
info "$(. /etc/os-release; echo $PRETTY_NAME) | kernel $(uname -r)"
info "system python3: $(python3 --version 2>&1)"
info "ROS_DISTRO=${ROS_DISTRO:-<not sourced>}"
command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1 | sed 's/^/  [info] GPU: /'
command -v pdflatex >/dev/null && ok "pdflatex present" || bad "pdflatex missing (report won't compile): sudo apt install texlive-latex-extra"
command -v pipx >/dev/null && ok "pipx $(pipx --version)" || bad "pipx missing"
VENVS=$(pipx environment --value PIPX_LOCAL_VENVS 2>/dev/null)
echo "== pipx tools"
for t in kiss-icp kiss-slam evo; do
  PY="$VENVS/$t/bin/python"
  if [ ! -x "$PY" ] || ! "$PY" -c 'pass' 2>/dev/null; then bad "$t: venv broken or missing (its Python was probably removed by the upgrade)"; continue; fi
  if V=$("$PY" -c 'import sys,numpy; print(f"py{sys.version_info.major}.{sys.version_info.minor} numpy {numpy.__version__}")' 2>/dev/null); then ok "$t venv runs ($V)"
  else bad "$t venv runs on $("$PY" --version 2>&1) but its packages are missing (installed for another Python)"; fi
done
P="$VENVS/kiss-icp/bin/python"; "$P" -c 'import kiss_icp;print(kiss_icp.__version__)' >/dev/null 2>&1 && ok "kiss_icp $("$P" -c 'import kiss_icp;print(kiss_icp.__version__)')" || bad "kiss_icp does not import"
"$P" -c 'import yaml' 2>/dev/null && ok "kiss-icp has pyyaml" || bad "kiss-icp missing pyyaml: pipx inject kiss-icp pyyaml"
"$P" -c 'import scipy' 2>/dev/null && ok "kiss-icp has scipy (NCLT loader)" || bad "kiss-icp missing scipy (NCLT loader needs it): pipx inject kiss-icp scipy"
F=$(ls "$VENVS"/kiss-icp/lib/python3*/site-packages/kiss_icp/pipeline.py 2>/dev/null | head -1)
[ -n "$F" ] && grep -q "np.ravel" "$F" && ok "KISS-ICP TUM-export patch present" || bad "KISS-ICP TUM-export patch missing (docs/pipeline_patch.diff)"
P="$VENVS/kiss-slam/bin/python"; "$P" -c 'import kiss_slam' 2>/dev/null && ok "kiss_slam $("$P" -m pip show kiss-slam 2>/dev/null | awk '/^Version/{print $2}')" || bad "kiss_slam does not import"
for m in PIL:pillow matplotlib:matplotlib scipy:scipy; do i=${m%%:*}; n=${m##*:}; "$P" -c "import $i" 2>/dev/null && ok "kiss-slam has $n" || bad "kiss-slam missing $n: pipx inject kiss-slam $n"; done
F=$(ls "$VENVS"/kiss-slam/lib/python3*/site-packages/kiss_slam/slam.py 2>/dev/null | head -1)
[ -n "$F" ] && grep -q "KS_ODOM_INFO" "$F" && ok "KISS-SLAM weighting patch present (needed for every W run)" || bad "KISS-SLAM weighting patch MISSING: W runs would silently use np.eye(6)"
"$VENVS/evo/bin/python" -c 'import evo' 2>/dev/null && ok "evo $("$VENVS/evo/bin/python" -m pip show evo 2>/dev/null | awk '/^Version/{print $2}')" || bad "evo does not import"
echo "== Phase 2.2 fusion venv (phase2_multisensor/.venv)"
FPY="$ROOT/.venv/bin/python"
if [ -x "$FPY" ] && V=$("$FPY" -c 'import sys,gtsam,numpy,scipy,importlib.metadata as m; print(f"py{sys.version_info.major}.{sys.version_info.minor} gtsam {m.version("gtsam")} numpy {numpy.__version__} scipy {scipy.__version__}")' 2>/dev/null); then
  ok "fusion venv runs ($V)"
else bad "fusion venv missing or broken: uv venv .venv --python 3.12 && uv pip install --python .venv/bin/python -r requirements-fusion.txt"; fi
echo "== Data"
for s in 2012-01-15 2012-02-12 2012-04-29 2012-05-26 2012-06-15 2012-08-04 2012-10-28 2013-01-10 2013-04-05; do
  [ -d "$ROOT/data/nclt/$s" ] || bad "session $s missing"; done
ok "NCLT sessions checked"
if [ "$1" = "--smoke" ] && [ $FAILS -eq 0 ]; then
  echo "== Smoke run (200 frames, KISS-ICP)"
  SMOKE=$(mktemp -d); cd "$SMOKE"
  kiss_icp_pipeline --dataloader nclt "$ROOT/data/nclt/2013-01-10" --n-scans 200 > smoke.log 2>&1
  if ls results/*/*_poses*.npy >/dev/null 2>&1 || ls results/*/*poses* >/dev/null 2>&1; then ok "smoke run produced poses"
  else bad "smoke run failed:"; grep -v SyntaxWarning smoke.log | tail -5 | sed 's/^/         /'; fi
  cd - >/dev/null
fi
echo; [ $FAILS -eq 0 ] && echo "ALL CHECKS PASSED" || echo "$FAILS CHECK(S) FAILED"
