#!/usr/bin/env bash
set -Eeuo pipefail

# Persist the logs and actual camera evidence even when the one-off container
# exits. The runner mounts /recordings from the project's recordings folder.
output=${1:-/recordings/landing-check-$(date -u +%Y%m%dT%H%M%SZ)-$$}
mkdir -p "$output"
output=$(cd "$output" && pwd)
launch_pid=
stage='resolving the ROS package'
cleanup() {
  local result=$?
  trap - EXIT
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for ((i=0; i<40; i++)); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ -f /root/.gz/rendering/ogre2.log ]]; then
    cp /root/.gz/rendering/ogre2.log "$output/ogre2.log" || true
  fi
  if [[ "$result" != 0 ]]; then
    printf 'Landing test failed while %s. Evidence: %s\n' "$stage" "$output" >&2
    for name in generation validation simulation; do
      if [[ -s "$output/$name.log" ]]; then
        printf '%s log (last 100 lines):\n' "$name" >&2
        tail -n 100 "$output/$name.log" >&2 || true
      fi
    done
  fi
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

export GZ_PARTITION="ocean-landing-test-$(date -u +%s)-$$"
export ROS_DOMAIN_ID="${LANDING_TEST_ROS_DOMAIN_ID:-32}"
# Headless still renders the real camera through Ogre2 and WSLg. Do not gate
# this diagnostic on GUI checks, so rendering failures can be reported in logs.
export REQUIRE_GPU=0
share=$(ros2 pkg prefix --share ocean_sim)
stage='generating the selected AprilTag and landing scene'
python3 - "$share" "$output" > "$output/generation.log" 2>&1 <<'PY'
import os
from pathlib import Path
import sys
import yaml
from ocean_sim.scene_builder import build_scene, load_configuration

share, output = Path(sys.argv[1]), Path(sys.argv[2])
config = load_configuration(share)
for key, converter in [('tag_family', str), ('tag_id', int), ('tag_size', float)]:
    value = os.environ.get(key.upper(), '')
    if value:
        config['scene'][key] = converter(value)
build_scene(share, output / 'generated', config['scene'])
(output / 'test-config.yaml').write_text(yaml.safe_dump(config), encoding='utf-8')
print('Generated matching tag texture, world, and controller configuration.')
PY
stage='validating the landing world SDF'
world="$output/generated/landing.sdf"
if ! SDF_PATH="$output/generated/models:${GZ_SIM_RESOURCE_PATH:-}${SDF_PATH:+:${SDF_PATH}}" \
  gz sdf -k "$world" > "$output/validation.log" 2>&1; then
  printf 'Landing world validation failed: %s\n' "$world" >&2
  exit 1
fi
stage='running Gazebo and verifying the camera-guided landing'
ros2 launch ocean_sim landing.launch.py gui:=false config:="$output/test-config.yaml" \
  > "$output/simulation.log" 2>&1 &
launch_pid=$!
python3 /opt/ocean/container/landing_probe.py --output "$output" \
  --wall-timeout "${LANDING_TEST_WALL_TIMEOUT:-180}" \
  --sim-timeout "${LANDING_TEST_SIM_TIMEOUT:-100}" \
  --settle-seconds "${LANDING_TEST_SETTLE_SECONDS:-2}" \
  | tee "$output/probe.log"
if grep -Eqi 'Failed to load (system )?plugin|Could not find (shared library|resource)|Unable to find uri|No wavefield found|Invalid wavefield|Assertion.*failed|Segmentation fault' "$output/simulation.log"; then
  echo 'A plugin, resource or Gazebo process failed; see simulation.log.' >&2
  exit 1
fi
echo "PASS: landing test completed. Actual camera images and JSON report: $output"
