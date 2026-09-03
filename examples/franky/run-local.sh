#!/usr/bin/env bash
#
# Run .github/workflows/franky.yml on a dev box.
#
#   examples/franky/run-local.sh [--start-sim] [--args '...'] [--reflex]
#
# The venv is $VENV (default <repo>/.validate/franky-venv, gitignored) and is
# reused if it already has the pinned franky-control. Without --start-sim the
# script assumes a franka-sim already serving the FCI on 127.0.0.1:1337.
#
# Environment:
#   VENV               virtualenv directory
#   FRANKA_SIM_IMAGE   sim image (default ghcr.io/barisyazici/franka-sim:latest)
#   FRANKY_VERSION     franky-control version (default: read from the workflow)

set -euo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
WORKFLOW="$REPO_ROOT/.github/workflows/franky.yml"
HERE="$REPO_ROOT/examples/franky"

VENV="${VENV:-$REPO_ROOT/.validate/franky-venv}"
FRANKA_SIM_IMAGE="${FRANKA_SIM_IMAGE:-ghcr.io/barisyazici/franka-sim:latest}"
SIM_CONTAINER=franka-sim
SIM_LOG="$REPO_ROOT/.validate/franky-sim.log"
SIM_WAIT=90
ROBOT_IP=127.0.0.1

START_SIM=0
RUN_REFLEX=0
SIM_ARGS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --start-sim) START_SIM=1 ;;
    --reflex) RUN_REFLEX=1 ;;
    --args) SIM_ARGS="${2:-}"; shift ;;
    -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# The pinned version lives in the workflow; this reads it from there so the two
# never drift.
FRANKY_VERSION="${FRANKY_VERSION:-$(sed -n \
  's/^[[:space:]]*FRANKY_VERSION:[[:space:]]*['\''"]\{0,1\}\([0-9][^'\''" ]*\).*$/\1/p' \
  "$WORKFLOW" | head -n 1)}"
if [ -z "$FRANKY_VERSION" ]; then
  echo "could not read FRANKY_VERSION from $WORKFLOW" >&2
  exit 1
fi

TOTAL_START=$SECONDS
hms() { printf '%dm%02ds' $(($1 / 60)) $(($1 % 60)); }

STEP_LABEL=""
STEP_START=0
step() { STEP_LABEL="$1"; STEP_START=$SECONDS; printf '\n=== %s ===\n' "$1"; }
step_done() { printf '=== %s: %s ===\n' "$STEP_LABEL" "$(hms $((SECONDS - STEP_START)))"; }

on_exit() {
  local rc=$?
  if [ "$START_SIM" = 1 ]; then
    # Logs first: removing the container takes them with it.
    docker logs "$SIM_CONTAINER" >"$SIM_LOG" 2>&1 || true
    docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
    echo "franka-sim log: $SIM_LOG"
  fi
  printf 'total: %s (exit %d)\n' "$(hms $((SECONDS - TOTAL_START)))" "$rc"
}

echo "venv:      $VENV"
echo "franky:    franky-control==$FRANKY_VERSION"
echo "sim image: $FRANKA_SIM_IMAGE"
echo "sim args:  ${SIM_ARGS:-<none>}"

mkdir -p "$(dirname "$SIM_LOG")"
trap on_exit EXIT

# --- 1. the venv -------------------------------------------------------------

step "python environment"
[ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
if ! "$VENV/bin/python" -c "
import sys
from importlib.metadata import version
sys.exit(0 if version('franky-control') == '$FRANKY_VERSION' else 1)
" 2>/dev/null; then
  "$VENV/bin/pip" install --quiet --upgrade pip
  "$VENV/bin/pip" install --quiet "franky-control==$FRANKY_VERSION"
fi
"$VENV/bin/python" -c "import franky; print('franky import ok')"
step_done

# --- 2. simulated robot ------------------------------------------------------

if [ "$START_SIM" = 1 ]; then
  step "start franka-sim"
  docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
  # Host networking: the FCI is a 1 kHz UDP loop, and published ports would put
  # Docker's NAT in the middle of it. SIM_ARGS is an argv tail, so it is
  # word-split on purpose.
  # shellcheck disable=SC2086
  docker run -d --network host --name "$SIM_CONTAINER" "$FRANKA_SIM_IMAGE" $SIM_ARGS >/dev/null
  deadline=$((SECONDS + SIM_WAIT))
  # franka-sim-check does a real Connect handshake and waits for a RobotState
  # datagram, so a pass means the FCI is genuinely being served. This is what
  # the BarisYazici/libfranka-sim action does for you in the workflow.
  until docker exec "$SIM_CONTAINER" franka-sim-check --timeout 5 >/dev/null 2>&1; do
    if [ "$SECONDS" -ge "$deadline" ] ||
       [ "$(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null)" != "true" ]; then
      echo "franka-sim did not become ready within ${SIM_WAIT}s" >&2
      docker logs "$SIM_CONTAINER" >&2 || true
      exit 1
    fi
    sleep 2
  done
  echo "franka-sim ready on $ROBOT_IP:1337"
  step_done
else
  echo "assuming a franka-sim already on $ROBOT_IP:1337 (pass --start-sim to run one)"
fi

# --- 3. the examples ---------------------------------------------------------

step "smoke.py"
timeout 120 "$VENV/bin/python" "$HERE/smoke.py" --host "$ROBOT_IP"
step_done

if [ "$RUN_REFLEX" = 1 ]; then
  step "reflex.py"
  timeout 120 "$VENV/bin/python" "$HERE/reflex.py" --host "$ROBOT_IP"
  step_done
fi
