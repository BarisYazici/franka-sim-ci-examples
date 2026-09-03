#!/usr/bin/env bash
#
# Run .github/workflows/franka_ros2.yml on a dev box.
#
#   examples/franka_ros2/run-local.sh [--start-sim]
#
# The workspace is $WS (default <repo>/.validate/franka_ros2, gitignored).
# Without --start-sim the script assumes a franka-sim already serving the FCI
# on 127.0.0.1:1337.
#
# Environment:
#   WS                        workspace directory
#   FRANKA_SIM_IMAGE          sim image (default ghcr.io/barisyazici/franka-sim:edge)
#   FRANKA_ROS2_IMAGE         tag for the built toolchain image (default franka_ros2:ci)
#   FRANKA_ROS2_DOCKERFILE    alternative Dockerfile for that build
#   FRANKA_ROS2_GIT_PROTOCOL0 =1: work around hosts whose containers cannot
#                             speak git protocol v2 to github.com (generates
#                             the overlay Dockerfile and pre-imports the source
#                             dependencies on the host, where git works)
#
# The container command below is the same string the workflow runs; keep the
# two in sync (a `diff` of WORKSPACE_SCRIPT against the workflow's heredoc
# should show comment differences only).

set -euo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
WORKFLOW="$REPO_ROOT/.github/workflows/franka_ros2.yml"

WS="${WS:-$REPO_ROOT/.validate/franka_ros2}"
FRANKA_SIM_IMAGE="${FRANKA_SIM_IMAGE:-ghcr.io/barisyazici/franka-sim:edge}"
FRANKA_ROS2_IMAGE="${FRANKA_ROS2_IMAGE:-franka_ros2:ci}"
FRANKA_ROS2_GIT_PROTOCOL0="${FRANKA_ROS2_GIT_PROTOCOL0:-0}"
FRANKA_ROS2_URL="https://github.com/frankarobotics/franka_ros2.git"
SIM_CONTAINER=franka-sim
SIM_WAIT=90
ROBOT_IP=127.0.0.1

START_SIM=0
for arg in "$@"; do
  case "$arg" in
    --start-sim) START_SIM=1 ;;
    -h|--help) sed -n '2,26p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

# --- step timing -------------------------------------------------------------

STEP_LABEL=""
STEP_START=0
TOTAL_START=$SECONDS

hms() {
  local s=$1
  printf '%dm%02ds' $((s / 60)) $((s % 60))
}

step() {
  STEP_LABEL="$1"
  STEP_START=$SECONDS
  printf '\n=== %s ===\n' "$STEP_LABEL"
}

step_done() {
  printf '=== %s: %s ===\n' "$STEP_LABEL" "$(hms $((SECONDS - STEP_START)))"
}

on_exit() {
  local rc=$?
  if [ "$START_SIM" = 1 ]; then
    # Logs first: `docker rm -f` on a --rm container takes them with it.
    docker logs "$SIM_CONTAINER" >"$WS/franka-sim.log" 2>&1 || true
    docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
    echo "franka-sim log: $WS/franka-sim.log"
  fi
  printf 'total: %s (exit %d)\n' "$(hms $((SECONDS - TOTAL_START)))" "$rc"
}

# --- the pinned upstream revision, read from the workflow --------------------

FRANKA_ROS2_REF=$(sed -n 's/^[[:space:]]*FRANKA_ROS2_REF:[[:space:]]*\([0-9a-fA-F]\{7,40\}\).*$/\1/p' \
  "$WORKFLOW" | head -n 1)
if [ -z "$FRANKA_ROS2_REF" ]; then
  echo "could not read FRANKA_ROS2_REF from $WORKFLOW" >&2
  exit 1
fi
echo "workspace:      $WS"
echo "franka_ros2:    $FRANKA_ROS2_REF"
echo "sim image:      $FRANKA_SIM_IMAGE"
echo "toolchain image: $FRANKA_ROS2_IMAGE"

mkdir -p "$WS/src"
trap on_exit EXIT

# --- 1. upstream sources -----------------------------------------------------

step "checkout franka_ros2"
SRC="$WS/src/franka_ros2"
if [ "$(git -C "$SRC" rev-parse HEAD 2>/dev/null || true)" = "$FRANKA_ROS2_REF" ]; then
  echo "already at $FRANKA_ROS2_REF, skipping fetch"
else
  if [ ! -d "$SRC/.git" ]; then
    rm -rf "$SRC"
    git clone "$FRANKA_ROS2_URL" "$SRC"
  else
    git -C "$SRC" fetch origin
  fi
  git -C "$SRC" checkout --detach "$FRANKA_ROS2_REF"
fi
step_done

# --- 2. simulated robot ------------------------------------------------------

if [ "$START_SIM" = 1 ]; then
  step "start franka-sim"
  docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
  # Host networking: the FCI is a 1 kHz UDP loop, and published ports would put
  # Docker's NAT in the middle of it.
  docker run -d --network host --name "$SIM_CONTAINER" "$FRANKA_SIM_IMAGE" >/dev/null
  deadline=$((SECONDS + SIM_WAIT))
  # franka-sim-check does a real Connect handshake and waits for a RobotState
  # datagram, so a pass means the FCI is genuinely being served.
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

# --- 3. git protocol v0 workaround -------------------------------------------

if [ "$FRANKA_ROS2_GIT_PROTOCOL0" = 1 ]; then
  step "git protocol v0 workaround"
  # Some hosts' containers cannot negotiate git protocol v2 against github.com
  # ("could not read Username ... expected flush after ref listing"). Two
  # halves: the image build's own `vcs import` gets protocol.version 0 forced
  # system-wide, and the workspace import is done here on the host, where git
  # works, so the in-container `--skip-existing` import is a no-op.
  if [ -z "${FRANKA_ROS2_DOCKERFILE:-}" ]; then
    # `vcs import` sits on a continuation line of a multi-command RUN, so walk
    # back to the RUN that owns it and insert the config in front of it.
    awk '
      { line[NR] = $0 }
      /vcs[[:space:]]+import/ && !target { target = NR }
      END {
        for (i = target; i >= 1; i--)
          if (line[i] ~ /^[[:space:]]*RUN[[:space:]]/) { insert = i; break }
        for (i = 1; i <= NR; i++) {
          if (insert && i == insert)
            print "RUN sudo git config --system protocol.version 0"
          print line[i]
        }
      }
    ' "$SRC/Dockerfile" >"$WS/Dockerfile.local"
    if ! grep -q 'protocol.version 0' "$WS/Dockerfile.local"; then
      echo "no 'vcs import' RUN line found in $SRC/Dockerfile; overlay not generated" >&2
      exit 1
    fi
    FRANKA_ROS2_DOCKERFILE="$WS/Dockerfile.local"
    echo "overlay Dockerfile: $FRANKA_ROS2_DOCKERFILE"
  fi

  if ! command -v vcs >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    if [ -r /home/yazi_ba/miniforge3/etc/profile.d/conda.sh ]; then
      . /home/yazi_ba/miniforge3/etc/profile.d/conda.sh
      conda activate libfranka-sim
    fi
    command -v vcs >/dev/null 2>&1 || pip install --user vcstool
  fi
  # vcstool still contacts every remote for repositories it already has, and
  # GitHub rate-limits unauthenticated fetches; on a warm workspace skip the
  # import entirely once every repository named in dependency.repos is present.
  missing=0
  while IFS= read -r name; do
    [ -d "$WS/src/$name" ] || missing=1
  done < <(sed -n 's/^  \([^ :][^:]*\):$/\1/p' "$SRC/dependency.repos")
  if [ "$missing" -eq 0 ]; then
    echo "all dependency.repos repositories present, skipping host-side vcs import"
  else
    vcs import "$WS/src" <"$SRC/dependency.repos" --recursive --skip-existing
  fi
  step_done
fi

# --- 4. the toolchain image --------------------------------------------------

step "build $FRANKA_ROS2_IMAGE"
# Upstream's Dockerfile installs the apt dependencies and then deletes its copy
# of the source tree: the image is a toolchain, not a workspace. The sources
# come from the bind mount in step 5.
docker build -t "$FRANKA_ROS2_IMAGE" \
  ${FRANKA_ROS2_DOCKERFILE:+-f "$FRANKA_ROS2_DOCKERFILE"} \
  "$SRC"
step_done

# --- 5. build the workspace and run the hardware tests -----------------------

# Keep line-for-line identical to the workflow's heredoc, modulo comments.
WORKSPACE_SCRIPT=$(cat <<'EOS'
set -eo pipefail
cd /ros
. /opt/ros/jazzy/setup.sh

vcs import src < src/franka_ros2/dependency.repos --recursive --skip-existing

if git -C src/ros2_control apply --check src/franka_ros2/patches/manage_overruns.patch 2>/dev/null; then
  git -C src/ros2_control apply src/franka_ros2/patches/manage_overruns.patch
fi

# Stop the hardware before deactivating controllers at shutdown

# (see examples/franka_ros2/README.md, 'Two lanes'): without it

# franka_hardware re-sends the last setpoint for one cycle after the

# controller stops, a velocity step the sim reports as a discontinuity.

# Pending upstream in ros2_control; applied here the same way.

if git -C src/ros2_control apply --check "$PATCH_DIR/stop-hardware-before-shutdown.patch" 2>/dev/null; then

  git -C src/ros2_control apply "$PATCH_DIR/stop-hardware-before-shutdown.patch"

fi

IGNORE=(franka_mobile franka_gazebo_bringup franka_mobile_sensors
        franka_vision_and_manipulation_kit franka_mobile_fr3_duo_moveit_config)

colcon build \
  --packages-up-to franka_bringup \
  --packages-ignore "${IGNORE[@]}" \
  --cmake-args -DBUILD_TESTING=ON -DCHECK_TIDY=OFF -DROBOT_IP="$ROBOT_IP"

. install/setup.sh

colcon test --packages-select franka_bringup \
  --packages-ignore "${IGNORE[@]}" \
  --ctest-args -R test_hardware \
  --event-handlers console_direct+ \
  --return-code-on-test-failure

colcon test-result --verbose
EOS
)

step "colcon build + hardware tests"
# --network host so 127.0.0.1 means the same thing here and in the sim;
# --entrypoint /bin/bash because the image entrypoint would re-run `vcs import`
# into /ros2_ws, which is not the mounted workspace.
docker run --rm --network host --entrypoint /bin/bash \
  -e ROBOT_IP="$ROBOT_IP" -e PATCH_DIR=/patches \
  -v "$REPO_ROOT/examples/franka_ros2/patches:/patches:ro" \
  -v "$WS:/ros" "$FRANKA_ROS2_IMAGE" -c "$WORKSPACE_SCRIPT"
step_done
