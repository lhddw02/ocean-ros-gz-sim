#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts are not compatible with bash nounset.
source /opt/ros/humble/setup.bash
source /opt/asv_ws/install/setup.bash
source /opt/ocean_ws/install/setup.bash

asv_models=/opt/asv_ws/src/asv_wave_sim/gz-waves-models
ocean_share=/opt/ocean_ws/install/share/ocean_sim
export GZ_SIM_RESOURCE_PATH="${ocean_share}/models:${ocean_share}/worlds:${asv_models}/models:${asv_models}/world_models:${asv_models}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ocean_ws/install/lib:/opt/asv_ws/install/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_GUI_PLUGIN_PATH="/opt/ocean_ws/install/lib${GZ_GUI_PLUGIN_PATH:+:${GZ_GUI_PLUGIN_PATH}}"
export GZ_RENDERING_PLUGIN_PATH="/opt/asv_ws/install/lib${GZ_RENDERING_PLUGIN_PATH:+:${GZ_RENDERING_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="/usr/lib/wsl/lib:/opt/asv_ws/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="${PATH}:/usr/lib/wsl/lib"

# A GUI launch is gated by an actual OpenGL GPU check. Explicit commands such
# as diagnostics, bash, and the headless smoke test remain available to repair
# a broken graphics configuration.
gui_launch=0
if [[ $# -eq 0 ]]; then
  set -- ros2 launch ocean_sim ocean.launch.py
fi
if [[ ${1:-} == ros2 && ${2:-} == launch && ${3:-} == ocean_sim ]]; then
  gui_launch=1
  for arg in "$@"; do
    [[ "$arg" == gui:=false ]] && gui_launch=0
  done
fi
if [[ "$gui_launch" == 1 && ${REQUIRE_GPU:-1} == 1 ]]; then
  /opt/ocean/container/check-gpu.sh
fi
exec "$@"
