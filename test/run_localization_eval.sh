#!/bin/bash
# Copyright 2026 emcl2_ros2 developers
# SPDX-FileCopyrightText: 2026 emcl2_ros2 developers
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# Localization evaluation: run headless sim + emcl2, drive a fixed path,
# record ground-truth and estimated poses, and compare them with evo.
#
# Env vars:
#   HEADLESS_RENDERING=true|false  use gz EGL software rendering (default: true)
#   OUTPUT_DIR=<path>              where evo results/plots are written
#   RETRIES=<n>                    bringup attempts before giving up (default: 3)
#
# Requires: this workspace sourced, nav2_minimal_tb3_sim, nav2_bringup, evo.
set -e

HEADLESS_RENDERING="${HEADLESS_RENDERING:-true}"
OUTPUT_DIR="${OUTPUT_DIR:-$(pwd)/localization_eval_output}"
RETRIES="${RETRIES:-3}"
# SCENARIO=tracking : APE against ground truth from a correct initial pose.
# SCENARIO=recovery : the estimate starts far from the truth (a wrong initial
#                     pose, or -- with TRIGGER_GLOBAL_LOC=true -- a uniform
#                     global reset) and we measure convergence instead of APE.
SCENARIO="${SCENARIO:-tracking}"
# Optional emcl2 param file forwarded to the launch (empty = launch default).
PARAMS_FILE="${PARAMS_FILE:-}"
# When true, call /reinitialize_global_localization once emcl2 is up, scattering
# the particles uniformly over free space (global-localization scenario).
TRIGGER_GLOBAL_LOC="${TRIGGER_GLOBAL_LOC:-false}"
# Translation error (m) under which the recovery scenario counts as converged.
CONVERGE_THRESHOLD="${CONVERGE_THRESHOLD:-0.3}"
WORK="$(mktemp -d)"
mkdir -p "$OUTPUT_DIR"

SIM=$(ros2 pkg prefix --share nav2_minimal_tb3_sim)
EMCL2=$(ros2 pkg prefix --share emcl2)

echo "[eval] HEADLESS_RENDERING=$HEADLESS_RENDERING  OUTPUT_DIR=$OUTPUT_DIR  RETRIES=$RETRIES"
echo "[eval] SCENARIO=$SCENARIO  TRIGGER_GLOBAL_LOC=$TRIGGER_GLOBAL_LOC  PARAMS_FILE=${PARAMS_FILE:-<launch default>}"

if [ "$HEADLESS_RENDERING" = "true" ]; then
  # GPU-less software rendering for gz's ogre2 sensors.
  export EGL_PLATFORM=surfaceless
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  unset DISPLAY || true
fi

# 1) Expand the world (headless, no SceneBroadcaster) and the robot SDF, then
#    inject a ground-truth OdometryPublisher into the robot.
xacro -o "$WORK/world.sdf" headless:=true "$SIM/worlds/tb3_sandbox.sdf.xacro"
xacro "$SIM/urdf/gz_waffle.sdf.xacro" > "$WORK/robot_plain.sdf"
python3 - "$WORK/robot_plain.sdf" "$WORK/robot_gt.sdf" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()
plugin = '''    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>map</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <dimensions>3</dimensions>
      <odom_topic>ground_truth</odom_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
'''
i = s.rfind('</model>')
open(dst, 'w').write(s[:i] + plugin + s[i:])
PY

# 2) Bring up sim + emcl2, retrying if localization does not actually start.
#    A transient lifecycle/map startup race can leave emcl2 advertising
#    /mcl_pose while never publishing (the particle filter never gets the map),
#    which would silently produce an all-zero estimate. We therefore wait for
#    real messages -- not just the topic -- and restart the whole bringup on
#    failure instead of driving a path against a filter that never localized.
LAUNCH_PID=""
BRINGUP_LOG="$OUTPUT_DIR/bringup.log"
start_bringup() {  # $1 = attempt number (for a per-attempt log)
  # Keep one log per attempt so a failed attempt's log is not overwritten by the
  # retry; the canonical bringup.log is set to the winning attempt on success.
  BRINGUP_LOG="$OUTPUT_DIR/bringup_attempt${1:-1}.log"
  # Run the launch in its own process group (setsid) so the entire tree -- gz,
  # bridges, robot_state_publisher, map_server, lifecycle_manager, emcl2 -- can
  # be torn down together. Signalling only the launch PID leaves those children
  # orphaned; stale lifecycle_manager/emcl2 nodes then collide with the next
  # attempt (duplicate node names and bonds) and pile up across repeated runs.
  local params_arg=""
  [ -n "$PARAMS_FILE" ] && params_arg="params_file:=$PARAMS_FILE"
  setsid ros2 launch emcl2 localization_eval.launch.py \
    world_sdf:="$WORK/world.sdf" robot_sdf:="$WORK/robot_gt.sdf" \
    headless_rendering:="$HEADLESS_RENDERING" $params_arg > "$BRINGUP_LOG" 2>&1 &
  LAUNCH_PID=$!
}
kill_bringup() {
  # Kill the whole process group (setsid made LAUNCH_PID its leader). This is the
  # scoped teardown for this run's launch tree.
  [ -n "$LAUNCH_PID" ] && kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  # Best-effort sweep for stragglers that may have detached from the group (gz
  # notably) and the separately-started logger. NOTE: these patterns are not
  # scoped to this run, so do not launch a second eval on the same machine
  # concurrently -- it would kill the other run's processes too. Fine for CI
  # (isolated container) and sequential local use.
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  pkill -f "pose_logger.py" 2>/dev/null || true
  LAUNCH_PID=""
  sleep 3  # let the process group die and gz transport ports free
}
cleanup() {
  kill_bringup
  rm -rf "$WORK"
}
trap cleanup EXIT

# Block until one real message arrives on a topic (data, not just advertised).
# `ros2 topic echo --once` can exit immediately -- before the sim has advertised
# the topic, or when a stale ros2-daemon cache from a previous run reports no
# active publisher -- so a single call would give up prematurely and tear down a
# still-starting bringup. Poll it until the overall deadline instead.
wait_for_msg() {  # $1=topic  $2=timeout_s
  local deadline=$((SECONDS + $2))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if timeout 10 ros2 topic echo "$1" --once > /dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

GT_TUM="$OUTPUT_DIR/ground_truth.tum"
EST_TUM="$OUTPUT_DIR/mcl_pose.tum"

# Fraction of the ground-truth time span that the estimate overlaps. emcl2 can
# stall part way through a run (e.g. a TF/sim-time hiccup drops all scans), and
# comparing only the early overlap would yield a misleadingly low APE. Returns
# 0 when either trajectory is missing or empty.
coverage() {
  python3 - "$GT_TUM" "$EST_TUM" <<'PY'
import sys
def span(f):
    ts = []
    try:
        for line in open(f):
            line = line.strip()
            if not line:
                continue
            try:
                ts.append(float(line.split()[0]))
            except (ValueError, IndexError):
                # skip a malformed line (e.g. a partially flushed final row)
                continue
    except OSError:
        return None
    return (ts[0], ts[-1]) if ts else None
gt = span(sys.argv[1])
est = span(sys.argv[2])
if not gt or not est or gt[1] <= gt[0]:
    print("0.000")
    raise SystemExit
overlap = max(0.0, min(gt[1], est[1]) - max(gt[0], est[0]))
print(f"{overlap / (gt[1] - gt[0]):.3f}")
PY
}

# One full evaluation attempt: bring up, wait for real data, drive the path
# while logging, and require the estimate to span (>=90% of) the ground-truth
# window. Returns 0 only on a clean, fully-covered run.
run_attempt() {  # $1 = attempt number
  start_bringup "$1"
  echo "[eval] waiting for /ground_truth and /mcl_pose data ..."
  if ! { wait_for_msg /ground_truth 120 && wait_for_msg /mcl_pose 90; }; then
    echo "[eval] no data (localization did not come up)"
    return 1
  fi
  sleep 5  # let the filter settle at the initial pose

  : > "$GT_TUM"
  : > "$EST_TUM"
  ros2 run emcl2 pose_logger.py "$GT_TUM" "$EST_TUM" > "$OUTPUT_DIR/logger.log" 2>&1 &
  local log_pid=$!
  sleep 2

  if [ "$TRIGGER_GLOBAL_LOC" = "true" ]; then
    # Scatter the particles uniformly AFTER logging has started, so the metric
    # captures the full uniform -> converged transient rather than a belief that
    # already re-converged during an untimed settle.
    echo "[eval] triggering global localization (uniform reset) ..."
    ros2 service call /reinitialize_global_localization std_srvs/srv/Empty \
      > /dev/null 2>&1 || echo "[eval] WARN: reinitialize_global_localization call failed"
    sleep 1
  fi

  echo "[eval] driving fixed path ..."
  ros2 run emcl2 drive_path.py
  sleep 2
  kill -INT "$log_pid" 2>/dev/null || true
  sleep 2
  kill -9 "$log_pid" 2>/dev/null || true

  if [ ! -s "$GT_TUM" ] || [ ! -s "$EST_TUM" ]; then
    echo "[eval] pose logs are empty (gt=$(wc -l <"$GT_TUM" 2>/dev/null) est=$(wc -l <"$EST_TUM" 2>/dev/null))"
    return 1
  fi
  local cov
  cov=$(coverage)
  echo "[eval] gt samples=$(wc -l <"$GT_TUM")  est samples=$(wc -l <"$EST_TUM")  coverage=$cov"
  if ! python3 -c "import sys; sys.exit(0 if float('$cov') >= 0.9 else 1)"; then
    echo "[eval] estimate covers only $cov of ground truth (emcl2 stalled mid-run)"
    return 1
  fi
  return 0
}

ok=false
for attempt in $(seq 1 "$RETRIES"); do
  echo "[eval] run attempt $attempt/$RETRIES ..."
  if run_attempt "$attempt"; then
    ok=true
    # Expose the winning attempt's bringup log under the canonical name.
    cp -f "$OUTPUT_DIR/bringup_attempt${attempt}.log" "$OUTPUT_DIR/bringup.log" 2>/dev/null || true
    break
  fi
  echo "[eval] attempt $attempt failed; restarting bringup ..."
  kill_bringup
done
if [ "$ok" != true ]; then
  echo "[eval] ERROR: no valid localization run after $RETRIES attempts"
  exit 3
fi

# 4) Score the run.
export MPLBACKEND=Agg
RESULT_RC=0
if [ "$SCENARIO" = "recovery" ]; then
  # Recovery / global-localization: measure convergence, not average APE. The
  # estimate starts far from the truth, so an APE mean would be meaningless.
  echo "[eval] computing convergence metrics (threshold=${CONVERGE_THRESHOLD} m) ..."
  ros2 run emcl2 convergence_metrics.py \
    "$GT_TUM" "$EST_TUM" "$CONVERGE_THRESHOLD" > "$OUTPUT_DIR/convergence.txt"
  RESULT_RC=$?
  cat "$OUTPUT_DIR/convergence.txt"
  # Informational: a failure to converge is a reportable result, not an error;
  # only a broken metric computation (rc>=2) is treated as a hard failure.
  [ "$RESULT_RC" -ge 2 ] || RESULT_RC=0
else
  # Tracking: APE against ground truth (no alignment: gz world == emcl2 map).
  # evo writes ape_plot_map.png (trajectory colored by error) and ape_plot_raw.png.
  echo "[eval] running evo_ape ..."
  evo_ape tum "$GT_TUM" "$EST_TUM" \
    --t_max_diff 0.1 \
    --save_results "$OUTPUT_DIR/ape_results.zip" \
    --plot_mode xy --save_plot "$OUTPUT_DIR/ape_plot.png" \
    2>&1 | tee "$OUTPUT_DIR/ape_stats.txt"
  RESULT_RC=${PIPESTATUS[0]}
fi

echo "[eval] done (rc=$RESULT_RC). Artifacts in $OUTPUT_DIR"
# Disable the EXIT trap and clean up explicitly so that signals from the
# killed background processes cannot turn a successful run into a failure.
trap - EXIT
cleanup
disown -a 2>/dev/null || true
exit "$RESULT_RC"
