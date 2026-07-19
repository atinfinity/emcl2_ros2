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

# World xacro to simulate and the matching map (defaults: the tb3_sandbox world
# and its map). Override both together to evaluate on a different environment,
# e.g. emcl2's symmetric_room world for persistent multi-modality. ROBOT_{X,Y,YAW}
# place the ground-truth robot spawn (and must sit in that world's free space).
WORLD_XACRO="${WORLD_XACRO:-$SIM/worlds/tb3_sandbox.sdf.xacro}"
MAP_YAML="${MAP_YAML:-}"
ROBOT_X="${ROBOT_X:--2.0}"
ROBOT_Y="${ROBOT_Y:--0.5}"
ROBOT_YAW="${ROBOT_YAW:-0.0}"

# Mid-run kidnap: KIDNAP_AT seconds into the drive, teleport the robot (and thus
# ground truth) to KIDNAP_{X,Y,YAW} via the gz set_pose service, so emcl2 must
# detect the sudden mismatch and recover through Sensor Resetting. Empty = off.
# Score it with SCENARIO=recovery; convergence_metrics.py measures the recovery
# from the teleport (it detects the ground-truth jump automatically).
KIDNAP_AT="${KIDNAP_AT:-}"
KIDNAP_X="${KIDNAP_X:-1.0}"
KIDNAP_Y="${KIDNAP_Y:-1.0}"
KIDNAP_YAW="${KIDNAP_YAW:-1.0}"

# Map/world mismatch: make the simulated world and emcl2's map disagree, to
# probe robustness to stale/edited maps.
#   WORLD_ADD_BOXES="x,y,sx,sy;..."   inject box obstacles into the world only
#                                     (obstacles the map does not have).
#   MAP_ADD_OBSTACLES="x,y,sx,sy;..." draw obstacles onto the map only (phantom
#                                     obstacles the world does not have).
# Both take world-metre centres and footprints; default empty (matched map).
WORLD_ADD_BOXES="${WORLD_ADD_BOXES:-}"
MAP_ADD_OBSTACLES="${MAP_ADD_OBSTACLES:-}"

echo "[eval] HEADLESS_RENDERING=$HEADLESS_RENDERING  OUTPUT_DIR=$OUTPUT_DIR  RETRIES=$RETRIES"
echo "[eval] SCENARIO=$SCENARIO  TRIGGER_GLOBAL_LOC=$TRIGGER_GLOBAL_LOC  PARAMS_FILE=${PARAMS_FILE:-<launch default>}"
echo "[eval] WORLD_XACRO=$WORLD_XACRO  MAP_YAML=${MAP_YAML:-<launch default>}  ROBOT=($ROBOT_X,$ROBOT_Y,$ROBOT_YAW)"

if [ "$HEADLESS_RENDERING" = "true" ]; then
  # GPU-less software rendering for gz's ogre2 sensors.
  export EGL_PLATFORM=surfaceless
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
  unset DISPLAY || true
fi

# 1) Expand the world (headless, no SceneBroadcaster) and the robot SDF, then
#    inject a ground-truth OdometryPublisher into the robot.
xacro -o "$WORK/world.sdf" headless:=true "$WORLD_XACRO"
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

# 1b) Optionally desync the world and the map (map/world mismatch eval).
if [ -n "$WORLD_ADD_BOXES" ]; then
  echo "[eval] adding world-only boxes: $WORLD_ADD_BOXES"
  python3 - "$WORK/world.sdf" "$WORLD_ADD_BOXES" <<'PY'
import sys
world, spec = sys.argv[1], sys.argv[2]
s = open(world).read()
blocks = ''
for k, box in enumerate(spec.split(';')):
    box = box.strip()
    if not box:
        continue
    x, y, sx, sy = (float(v) for v in box.split(','))
    blocks += (
        '    <model name="unmapped_box_{k}"><static>1</static>\n'
        '      <link name="link"><pose>{x} {y} 0.25 0 0 0</pose>\n'
        '        <collision name="c"><geometry><box><size>{sx} {sy} 0.5</size></box>'
        '</geometry></collision>\n'
        '        <visual name="v"><geometry><box><size>{sx} {sy} 0.5</size></box>'
        '</geometry></visual>\n'
        '      </link></model>\n').format(k=k, x=x, y=y, sx=sx, sy=sy)
i = s.rfind('</world>')
open(world, 'w').write(s[:i] + blocks + s[i:])
PY
fi
if [ -n "$MAP_ADD_OBSTACLES" ]; then
  echo "[eval] adding map-only obstacles: $MAP_ADD_OBSTACLES"
  BASE_MAP="${MAP_YAML:-$(ros2 pkg prefix --share nav2_bringup)/maps/tb3_sandbox.yaml}"
  MAP_YAML=$(ros2 run emcl2 add_map_obstacles.py "$BASE_MAP" "$WORK" "$MAP_ADD_OBSTACLES" 2>/dev/null) ||
    { echo "[eval] ERROR: failed to build mismatch map"; exit 1; }
fi

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
  local map_arg=""
  [ -n "$MAP_YAML" ] && map_arg="map:=$MAP_YAML"
  setsid ros2 launch emcl2 localization_eval.launch.py \
    world_sdf:="$WORK/world.sdf" robot_sdf:="$WORK/robot_gt.sdf" \
    headless_rendering:="$HEADLESS_RENDERING" $params_arg $map_arg \
    robot_x:="$ROBOT_X" robot_y:="$ROBOT_Y" robot_yaw:="$ROBOT_YAW" \
    > "$BRINGUP_LOG" 2>&1 &
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

# Teleport the robot mid-run via the gz set_pose service (kidnapped robot).
kidnap_robot() {
  local qz qw
  qz=$(python3 -c "import math; print(math.sin($KIDNAP_YAW / 2.0))")
  qw=$(python3 -c "import math; print(math.cos($KIDNAP_YAW / 2.0))")
  echo "[eval] kidnapping robot to ($KIDNAP_X, $KIDNAP_Y, $KIDNAP_YAW) ..."
  gz service -s /world/default/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: 'turtlebot3_waffle', position: {x: $KIDNAP_X, y: $KIDNAP_Y, z: 0.01}, orientation: {z: $qz, w: $qw}" \
    > /dev/null 2>&1 || echo "[eval] WARN: kidnap set_pose failed"
}

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
  # For a wrong-initial-pose recovery (scenario B) the filter starts recovering
  # the instant it receives the map, so a long settle would hide the transient
  # before logging even begins. Use a short settle there; keep the normal settle
  # for tracking and for the global-localization scenario (whose scatter is
  # triggered only after logging has started).
  local settle=5
  if [ "$SCENARIO" = "recovery" ] && [ "$TRIGGER_GLOBAL_LOC" != "true" ]; then
    settle=1
  fi
  sleep "$settle"  # let the filter settle at the initial pose

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

  if [ -n "$KIDNAP_AT" ]; then
    # Fire the teleport KIDNAP_AT seconds into the drive, concurrently with it.
    ( sleep "$KIDNAP_AT"; kidnap_robot ) &
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

  # Also draw the trajectories over the occupancy map (evo's xy plot has no map),
  # using the map emcl2 actually localized against (the mismatch map if any).
  plot_map="${MAP_YAML:-$(ros2 pkg prefix --share nav2_bringup)/maps/tb3_sandbox.yaml}"
  ros2 run emcl2 plot_trajectory_on_map.py "$GT_TUM" "$EST_TUM" "$plot_map" \
    "$OUTPUT_DIR/trajectory_map.png" "emcl2 estimate vs ground truth on map" > /dev/null 2>&1 ||
    echo "[eval] WARN: trajectory-on-map plot failed"

  # Peak linear/angular speed and path length actually driven (from ground truth).
  ros2 run emcl2 motion_stats.py "$GT_TUM" > "$OUTPUT_DIR/motion_stats.txt" 2>/dev/null ||
    echo "[eval] WARN: motion stats failed"
  cat "$OUTPUT_DIR/motion_stats.txt" 2>/dev/null || true
fi

echo "[eval] done (rc=$RESULT_RC). Artifacts in $OUTPUT_DIR"
# Disable the EXIT trap and clean up explicitly so that signals from the
# killed background processes cannot turn a successful run into a failure.
trap - EXIT
cleanup
disown -a 2>/dev/null || true
exit "$RESULT_RC"
