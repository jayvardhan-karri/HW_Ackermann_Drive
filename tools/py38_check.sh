#!/usr/bin/env bash
# =============================================================================
# py38_check.sh - check this repo for the Foxy lab machine BEFORE pushing to
#                 the foxy-python38-support branch.
#
# Builds (once) a local venv .venv-py38/ with Python 3.8 and Ubuntu 20.04's
# library versions (numpy 1.17.4, scipy 1.3.3, matplotlib 3.1.2, PyYAML 5.3.1),
# then runs tools/py38_check/checks.py in it. No ROS needed.
#
# Usage   : tools/py38_check.sh            (create venv if missing, run checks)
#           tools/py38_check.sh --rebuild  (delete and rebuild the venv first)
# Needs   : uv (preferred: it downloads Python 3.8 itself) or python3.8 on PATH
# Exit    : 0 all checks passed, 1 a check failed, 2 the venv could not be made
# Not checked: real rclpy / DDS (see docs/COMPATIBILITY.md), so still run
#             tools/check_deps.py and hw_check on the Foxy machine.
# =============================================================================
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv-py38"
PINS=(numpy==1.17.4 scipy==1.3.3 matplotlib==3.1.2 PyYAML==5.3.1)

echo "================================================================"
echo "  Foxy / Python 3.8 check: $ROOT"
echo "================================================================"

if [ "${1:-}" = "--rebuild" ] && [ -d "$VENV" ]; then
  echo "## removing $VENV"; rm -r -- "$VENV"
fi

if [ ! -x "$VENV/bin/python" ]; then
  echo "## creating $VENV (one time)"
  if command -v uv >/dev/null 2>&1; then
    UV_LINK_MODE=copy uv -q venv --python 3.8 "$VENV" && UV_LINK_MODE=copy uv -q pip install --python "$VENV/bin/python" "${PINS[@]}" \
      || { echo "FAIL: could not build the venv with uv"; exit 2; }
  elif command -v python3.8 >/dev/null 2>&1; then
    python3.8 -m venv "$VENV" && "$VENV/bin/pip" install -q "${PINS[@]}" \
      || { echo "FAIL: could not build the venv with python3.8"; exit 2; }
  else
    echo "FAIL: need uv (https://docs.astral.sh/uv/) or python3.8 to build the venv"; exit 2
  fi
fi

echo "## running checks"
PYTHONDONTWRITEBYTECODE=1 "$VENV/bin/python" -W ignore "$ROOT/tools/py38_check/checks.py"
rc=$?
echo "================================================================"
[ $rc -eq 0 ] && echo "  OK for Foxy (Python 3.8). Next: push, then on the lab machine run hw_check." \
              || echo "  NOT ready: fix the FAIL lines above before pushing."
echo "================================================================"
exit $rc
