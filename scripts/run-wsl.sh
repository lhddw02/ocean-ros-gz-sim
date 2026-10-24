#!/usr/bin/env bash
set -Eeuo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
action="${1:-help}"

usage() {
  cat <<'USAGE'
Run this script in Ubuntu on WSL 2 with Docker Desktop WSL integration enabled.

  bash scripts/run-wsl.sh build   Build the ROS Humble + Gazebo ocean image.
  bash scripts/run-wsl.sh doctor  Check WSL, Docker, display and GPU prerequisites.
  bash scripts/run-wsl.sh check   Verify OpenGL uses the NVIDIA RTX 3060.
  bash scripts/run-wsl.sh start   Start Gazebo; reject CPU rendering.
  bash scripts/run-wsl.sh restart Recreate Gazebo after configuration changes.
  bash scripts/run-wsl.sh landing Run the moored station and AprilTag landing demo.
  bash scripts/run-wsl.sh landing-test Verify camera detection and physical landing.
  bash scripts/run-wsl.sh exec    Open a ROS shell in the running simulation.
  bash scripts/run-wsl.sh shell   Open a separate ROS-enabled container shell.
  bash scripts/run-wsl.sh smoke   Check the ocean simulation and ROS bridge.
  bash scripts/run-wsl.sh status  Show the simulation container status.
  bash scripts/run-wsl.sh config  Validate the resolved Compose configuration.
  bash scripts/run-wsl.sh stop    Stop and remove the simulation container.
  bash scripts/run-wsl.sh logs    Follow the simulation logs.

Optional environment variables: BUILD_JOBS=2, GPU_EXPECT=3060,
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA, ROS_DOMAIN_ID=30, OCEAN_DEV=0.
Use OCEAN_DEV=1 on every runner command to mount editable simulation files.
Use TAG_ID=7, TAG_FAMILY=tag36h11, TAG_SIZE=0.4 or AUTOLAND=false with landing.
ROS source in ./workspace and bags in ./recordings persist after stop.
USAGE
}

fail() {
  printf 'Cannot %s: %s\n' "$action" "$*" >&2
  exit 1
}

case "$action" in
  help|-h|--help) usage; exit 0 ;;
  build|doctor|check|start|restart|landing|landing-test|exec|shell|stop|logs|smoke|status|config) ;;
  *) usage >&2; fail "Unknown action '$action'." ;;
esac
[[ $# -le 1 ]] || fail 'Each action accepts no extra arguments; configure the documented environment variables instead.'

grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null || fail 'Run this from Ubuntu in WSL 2. In Windows, install WSL with: wsl --install -d Ubuntu-22.04'
command -v docker >/dev/null 2>&1 || fail 'Docker is unavailable in this WSL distribution. Install Docker Desktop for Windows and enable Settings > Resources > WSL Integration for Ubuntu.'
docker compose version >/dev/null 2>&1 || fail 'Docker Compose v2 is unavailable. Update Docker Desktop and enable WSL integration.'
if [[ "$action" != config ]]; then
  engine_os="$(docker info --format '{{.OSType}}' 2>/dev/null)" || fail 'Docker engine is unavailable. Start Docker Desktop, select Linux containers and enable its WSL 2 engine.'
  [[ "$engine_os" == linux ]] || fail 'Docker must be using Linux containers. Select the WSL 2 engine in Docker Desktop.'
fi

case "$action" in
  doctor|check|start|restart|landing|landing-test|shell|smoke)
    [[ -c /dev/dxg ]] || fail '/dev/dxg is missing. Use WSL 2 (wsl --list --verbose), update WSL (wsl --update) and install the Windows NVIDIA driver.'
    if [[ "$action" == doctor || "$action" == check || "$action" == start || "$action" == restart || "$action" == landing ]]; then
      [[ -n "${DISPLAY:-}" ]] || fail 'DISPLAY is unset. Reopen the Ubuntu WSL terminal after wsl --update; WSLg should supply DISPLAY automatically.'
    fi
    [[ -d /mnt/wslg && -d /tmp/.X11-unix ]] || fail 'WSLg display sockets are missing. Update WSL and reopen Ubuntu; Windows 10 build 19044 or Windows 11 is required.'
    [[ -r /usr/lib/wsl/lib/libd3d12.so ]] || fail 'WSL D3D12 libraries are missing. Update WSL and the Windows NVIDIA driver; reopen Ubuntu.'
    if [[ "$action" != doctor ]]; then
      docker image inspect ros-humble-ocean:harmonic >/dev/null 2>&1 || fail 'The simulation image has not been built. Run: bash scripts/run-wsl.sh build'
    fi
    ;;
esac

export BUILD_JOBS="${BUILD_JOBS:-2}"
[[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || fail 'BUILD_JOBS must be a positive integer.'
compose=(docker compose --project-directory "$project_root" -f "$project_root/compose.wsl.yaml")
if [[ "$action" == landing || "$action" == landing-test ]]; then
  compose+=(-f "$project_root/compose.landing.yaml")
fi
case "${OCEAN_DEV:-0}" in
  1) compose+=(-f "$project_root/compose.dev.yaml") ;;
  0) ;;
  *) fail 'OCEAN_DEV must be 0 or 1.' ;;
esac
mkdir -p "$project_root/workspace" "$project_root/recordings"

case "$action" in
  build) "${compose[@]}" build --pull ocean ;;
  doctor)
    docker version --format 'Docker client {{.Client.Version}}, engine {{.Server.Version}}'
    docker compose version
    "${compose[@]}" config --quiet
    printf 'PASS: WSL, Linux Docker engine, WSLg sockets and D3D12 device/libraries are available.\n'
    if docker image inspect ros-humble-ocean:harmonic >/dev/null 2>&1; then
      printf 'The image exists. Run check for actual OpenGL GPU verification.\n'
    else
      printf 'Next: bash scripts/run-wsl.sh build\n'
    fi
    ;;
  check) "${compose[@]}" run --rm --no-deps ocean /opt/ocean/container/check-gpu.sh ;;
  start|restart|landing)
    "${compose[@]}" run --rm --no-deps ocean /opt/ocean/container/check-gpu.sh
    "${compose[@]}" up --detach --no-build --force-recreate ocean
    printf 'Gazebo is starting. Follow startup and GPU verification with: bash scripts/run-wsl.sh logs\n'
    ;;
  shell) "${compose[@]}" run --rm --no-deps ocean bash ;;
  exec) "${compose[@]}" exec ocean /opt/ocean/container/entrypoint.sh bash ;;
  smoke) "${compose[@]}" run --rm --no-deps ocean /opt/ocean/container/smoke-test.sh ;;
  landing-test) "${compose[@]}" run --rm --no-deps ocean /opt/ocean/container/landing-test.sh ;;
  status) "${compose[@]}" ps ocean ;;
  config) "${compose[@]}" config --quiet; printf 'Compose configuration is valid.\n' ;;
  stop) "${compose[@]}" down ;;
  logs) "${compose[@]}" logs --follow ocean ;;
esac
