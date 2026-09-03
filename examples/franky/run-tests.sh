#!/usr/bin/env bash
#
# Run franky's test_integration_franky_sim.py against a franka-sim, one pytest
# process per test. Shared by the workflow and run-local.sh.
#
#   examples/franky/run-tests.sh TEST_FILE [PYTEST_ARGS...]
#
# One process per test rather than one for the file: libfranka allows a single
# FCI client, and when a test fails, its franky.Robot stays referenced from the
# traceback pytest keeps for the report, so the session never closes and every
# later test blocks in connect() -- inside C++, where pytest-timeout's signal
# cannot reach it. A process exit closes the socket unconditionally, and the
# outer `timeout` bounds a wedged test on its own.
#
# PYTEST_ARGS go to the collection pass that picks the tests (so `-k` and
# `--deselect` work) and to every per-test invocation.
#
# Environment:
#   PYTHON             interpreter (default: python)
#   FRANKA_SIM_HOST    address the franky_sim stand-in hands the tests (default 127.0.0.1)
#   PER_TEST_TIMEOUT   seconds before a test process is killed (default 150)

set -uo pipefail

if [ $# -lt 1 ]; then
  sed -n '2,21p' "${BASH_SOURCE[0]}"
  exit 2
fi
TEST_FILE=$1
shift

HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PYTHON="${PYTHON:-python}"
PER_TEST_TIMEOUT="${PER_TEST_TIMEOUT:-150}"

# franky_sim resolves to the stand-in in this directory; no __pycache__ in the
# tracked tree.
export PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
export FRANKA_SIM_HOST="${FRANKA_SIM_HOST:-127.0.0.1}"

# Collection only imports the file; nothing connects to the sim yet.
if ! collected=$("$PYTHON" -m pytest -p no:cacheprovider --collect-only -q "$TEST_FILE" "$@" 2>&1); then
  echo "$collected"
  exit 2
fi
mapfile -t TESTS < <(grep '::' <<<"$collected")
echo "${#TESTS[@]} tests selected"

passed=0
failed=()
start=$SECONDS
for nodeid in ${TESTS[@]+"${TESTS[@]}"}; do
  timeout -k 10 "$PER_TEST_TIMEOUT" "$PYTHON" -m pytest -p no:cacheprovider --no-header -v \
    --timeout=120 "$nodeid" "$@"
  rc=$?
  case $rc in
    0) passed=$((passed + 1)) ;;
    124|137) failed+=("$nodeid (timed out after ${PER_TEST_TIMEOUT}s)") ;;
    *) failed+=("$nodeid (exit $rc)") ;;
  esac
done

echo
echo "${#TESTS[@]} tests: $passed passed, ${#failed[@]} failed, in $((SECONDS - start))s"
for f in ${failed[@]+"${failed[@]}"}; do
  echo "  FAILED $f"
done
[ "${#failed[@]}" -eq 0 ]
