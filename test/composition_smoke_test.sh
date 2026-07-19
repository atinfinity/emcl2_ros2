#!/bin/bash
# Copyright 2026 emcl2_ros2 developers
# SPDX-FileCopyrightText: 2026 emcl2_ros2 developers
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# Composition smoke test: load emcl2::EMcl2Node into a component container and
# take it through the `configure` transition. This checks that the component is
# registered, links, constructs, and that on_configure() (pubs/subs/services/tf)
# runs — i.e. the composition path added for nav2 integration keeps working.
#
# Requires this workspace to be sourced. No Gazebo, map or scan is needed.
set -e

CONTAINER=emcl2_test_container
NODE=emcl2_composed

ros2 run rclcpp_components component_container --ros-args -r __node:="$CONTAINER" \
  > /tmp/emcl2_composition_container.log 2>&1 &
CC_PID=$!
cleanup() { kill "$CC_PID" 2>/dev/null || true; }
trap cleanup EXIT

echo "[smoke] waiting for the component container ..."
timeout 30 bash -c "until ros2 node list 2>/dev/null | grep -q '/$CONTAINER'; do sleep 1; done"

echo "[smoke] loading emcl2::EMcl2Node ..."
ros2 component load "/$CONTAINER" emcl2 emcl2::EMcl2Node -n "$NODE"

if ! ros2 component list 2>/dev/null | grep -q "$NODE"; then
  echo "[smoke] FAIL: $NODE not present in the container"
  exit 1
fi

# Wait for the lifecycle interface of the loaded node to come up.
timeout 15 bash -c "until ros2 lifecycle get /$NODE 2>/dev/null | grep -q .; do sleep 0.5; done"
echo "[smoke] state after load: $(ros2 lifecycle get /$NODE)"

echo "[smoke] configuring ..."
ros2 lifecycle set "/$NODE" configure
STATE=""
for _ in $(seq 1 20); do
  STATE=$(ros2 lifecycle get "/$NODE" 2>/dev/null || true)
  echo "$STATE" | grep -q "inactive" && break
  sleep 0.5
done
echo "[smoke] state after configure: $STATE"
if ! echo "$STATE" | grep -q "inactive"; then
  echo "[smoke] FAIL: node did not reach 'inactive' after configure"
  exit 1
fi

echo "[smoke] PASS: emcl2::EMcl2Node loaded and configured as a component"
trap - EXIT
cleanup
exit 0
