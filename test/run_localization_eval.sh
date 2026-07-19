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
WORK="$(mktemp -d)"
mkdir -p "$OUTPUT_DIR"

SIM=$(ros2 pkg prefix --share nav2_minimal_tb3_sim)
EMCL2=$(ros2 pkg prefix --share emcl2)

echo "[eval] HEADLESS_RENDERING=$HEADLESS_RENDERING  OUTPUT_DIR=$OUTPUT_DIR  RETRIES=$RETRIES"

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
start_bringup() {
  ros2 launch emcl2 localization_eval.launch.py \
    world_sdf:="$WORK/world.sdf" robot_sdf:="$WORK/robot_gt.sdf" \
    headless_rendering:="$HEADLESS_RENDERING" > "$OUTPUT_DIR/bringup.log" 2>&1 &
  LAUNCH_PID=$!
}
kill_bringup() {
  [ -n "$LAUNCH_PID" ] && kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  LAUNCH_PID=""
  sleep 3  # let gz transport ports free before a restart
}
cleanup() {
  kill_bringup
  rm -rf "$WORK"
}
trap cleanup EXIT

# Block until one real message arrives on a topic (data, not just advertised).
wait_for_msg() {  # $1=topic  $2=timeout_s
  timeout "$2" ros2 topic echo "$1" --once > /dev/null 2>&1
}

localized=false
for attempt in $(seq 1 "$RETRIES"); do
  echo "[eval] bringup attempt $attempt/$RETRIES ..."
  start_bringup
  echo "[eval] waiting for /ground_truth and /mcl_pose data ..."
  if wait_for_msg /ground_truth 120 && wait_for_msg /mcl_pose 90; then
    localized=true
    break
  fi
  echo "[eval] attempt $attempt did not localize (no data); restarting bringup ..."
  kill_bringup
done
if [ "$localized" != true ]; then
  echo "[eval] ERROR: localization did not come up after $RETRIES attempts"
  exit 3
fi
# let the filter settle
sleep 5

# 3) Log poses (TUM files, flushed per line) and drive the fixed path.
GT_TUM="$OUTPUT_DIR/ground_truth.tum"
EST_TUM="$OUTPUT_DIR/mcl_pose.tum"
ros2 run emcl2 pose_logger.py "$GT_TUM" "$EST_TUM" > "$OUTPUT_DIR/logger.log" 2>&1 &
LOG_PID=$!
sleep 2
echo "[eval] driving fixed path ..."
ros2 run emcl2 drive_path.py
sleep 2
kill -INT "$LOG_PID" 2>/dev/null || true
sleep 2
kill -9 "$LOG_PID" 2>/dev/null || true

if [ ! -s "$GT_TUM" ] || [ ! -s "$EST_TUM" ]; then
  echo "[eval] ERROR: pose logs are empty (gt=$(wc -l <"$GT_TUM" 2>/dev/null) est=$(wc -l <"$EST_TUM" 2>/dev/null))"
  exit 1
fi
echo "[eval] gt samples=$(wc -l <"$GT_TUM")  est samples=$(wc -l <"$EST_TUM")"

# 4) Compare with evo (no alignment: gz world frame == emcl2 map frame).
# Agg backend so the plots render without a display; evo writes
# ape_plot_map.png (trajectory colored by error) and ape_plot_raw.png.
export MPLBACKEND=Agg
echo "[eval] running evo_ape ..."
evo_ape tum "$GT_TUM" "$EST_TUM" \
  --t_max_diff 0.1 \
  --save_results "$OUTPUT_DIR/ape_results.zip" \
  --plot_mode xy --save_plot "$OUTPUT_DIR/ape_plot.png" \
  2>&1 | tee "$OUTPUT_DIR/ape_stats.txt"
EVO_RC=${PIPESTATUS[0]}

echo "[eval] done (evo rc=$EVO_RC). Artifacts in $OUTPUT_DIR"
# Disable the EXIT trap and clean up explicitly so that signals from the
# killed background processes cannot turn a successful run into a failure.
trap - EXIT
cleanup
disown -a 2>/dev/null || true
exit "$EVO_RC"
