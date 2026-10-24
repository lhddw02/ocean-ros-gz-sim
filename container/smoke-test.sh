#!/usr/bin/env bash
set -Eeuo pipefail

scratch=$(mktemp -d)
launch_pid=
stage='resolving the ROS package'
cleanup() {
  local result=$?
  trap - EXIT
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for ((i=0; i<20; i++)); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ "$result" != 0 ]]; then
    if [[ -s "$scratch/validation.log" ]]; then
      echo 'SDF validation log (last 100 lines):' >&2
      tail -n 100 "$scratch/validation.log" >&2 || true
    fi
    if [[ -f "$scratch/simulation.log" ]]; then
      echo 'Simulation log (last 100 lines):' >&2
      tail -n 100 "$scratch/simulation.log" >&2 || true
    else
      printf 'Gazebo was not started; smoke test failed while %s.\n' "$stage" >&2
    fi
  fi
  rm -rf "$scratch"
  exit "$result"
}
trap cleanup EXIT

# Isolate this test from an already running GUI simulation and ROS nodes.
export GZ_PARTITION="ocean-smoke-$$"
export ROS_DOMAIN_ID=31
export REQUIRE_GPU=0
share=$(ros2 pkg prefix --share ocean_sim)
stage='validating the world SDF'
# The standalone SDFormat CLI does not install Gazebo Sim's resource callback.
# Give this command the same local model roots through its own SDF_PATH lookup.
# Scope SDF_PATH to this command; Gazebo Sim still uses GZ_SIM_RESOURCE_PATH.
if ! SDF_PATH="${GZ_SIM_RESOURCE_PATH:-}${SDF_PATH:+:${SDF_PATH}}" \
  gz sdf -k "$share/worlds/ocean.sdf" > "$scratch/validation.log" 2>&1; then
  printf 'World validation failed. GZ_SIM_RESOURCE_PATH=%s\n' \
    "${GZ_SIM_RESOURCE_PATH:-<unset>}" >&2
  exit 1
fi
stage='running Gazebo and checking ROS data'
ros2 launch ocean_sim ocean.launch.py gui:=false verbosity:=3 \
  > "$scratch/simulation.log" 2>&1 &
launch_pid=$!
python3 /opt/ocean/container/smoke_probe.py
if grep -Eqi 'Failed to load (system )?plugin|Could not find (shared library|resource)|Unable to find uri|No wavefield found|Invalid wavefield' "$scratch/simulation.log"; then
  echo 'A plugin or resource failed to load.' >&2
  exit 1
fi
echo 'PASS: world loads, ROS simulation time advances, and the floating buoy moves vertically.'
