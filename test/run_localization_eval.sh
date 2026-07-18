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
#
# Requires: this workspace sourced, nav2_minimal_tb3_sim, nav2_bringup, evo.
set -e

HEADLESS_RENDERING="${HEADLESS_RENDERING:-true}"
OUTPUT_DIR="${OUTPUT_DIR:-$(pwd)/localization_eval_output}"
WORK="$(mktemp -d)"
mkdir -p "$OUTPUT_DIR"

SIM=$(ros2 pkg prefix --share nav2_minimal_tb3_sim)
EMCL2=$(ros2 pkg prefix --share emcl2)

echo "[eval] HEADLESS_RENDERING=$HEADLESS_RENDERING  OUTPUT_DIR=$OUTPUT_DIR"

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

# 2) Bring up sim + emcl2.
ros2 launch emcl2 localization_eval.launch.py \
  world_sdf:="$WORK/world.sdf" robot_sdf:="$WORK/robot_gt.sdf" \
  headless_rendering:="$HEADLESS_RENDERING" > "$OUTPUT_DIR/bringup.log" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  rm -rf "$WORK"
}
trap cleanup EXIT

# 3) Wait for localization to be up (both GT and estimate publishing).
echo "[eval] waiting for /ground_truth and /mcl_pose ..."
timeout 120 bash -c '
  until ros2 topic list 2>/dev/null | grep -q "^/ground_truth$" \
        && ros2 topic list 2>/dev/null | grep -q "^/mcl_pose$"; do sleep 1; done'
# let the filter settle
sleep 5

# 4) Log poses (TUM files, flushed per line) and drive the fixed path.
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

# 5) Compare with evo (no alignment: gz world frame == emcl2 map frame).
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
