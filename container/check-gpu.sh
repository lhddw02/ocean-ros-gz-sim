#!/usr/bin/env bash
set -Eeuo pipefail

fail() {
  printf 'GPU check failed: %s\n' "$*" >&2
  exit 1
}

[[ -n "${DISPLAY:-}" ]] || fail 'DISPLAY is unset. Run scripts/run-wsl.sh from the Ubuntu WSL 2 terminal.'
[[ -c /dev/dxg ]] || fail '/dev/dxg is missing. Enable WSL 2 and Docker Desktop GPU support.'
[[ -r /usr/lib/wsl/lib/libd3d12.so ]] || fail 'WSL D3D12 libraries are missing. Mount /usr/lib/wsl into the container.'
command -v glxinfo >/dev/null 2>&1 || fail 'glxinfo is missing from the image. Rebuild with mesa-utils installed.'

# Check actual OpenGL rendering; CUDA visibility alone cannot prove Gazebo uses the GPU.
if ! gpu_info="$(LC_ALL=C glxinfo -B 2>&1)"; then
  printf '%s\n' "$gpu_info" >&2
  fail 'OpenGL could not connect to WSLg. Check DISPLAY and the /tmp/.X11-unix and /mnt/wslg mounts.'
fi
printf '%s\n' "$gpu_info"

gpu_renderer="$(awk -F': ' '/^OpenGL renderer string:/{print $2; exit}' <<< "$gpu_info")"
[[ -n "$gpu_renderer" ]] || fail 'glxinfo did not report an OpenGL renderer.'
[[ "${gpu_renderer,,}" != *llvmpipe* && "${gpu_renderer,,}" != *softpipe* ]] || fail 'Mesa selected CPU rendering. Update WSL and the Windows NVIDIA driver, then rebuild the image.'
[[ "${gpu_renderer,,}" == *d3d12* && "${gpu_renderer,,}" == *nvidia* ]] || fail "Expected the WSLg D3D12 NVIDIA renderer, received: $gpu_renderer"
gpu_expect="${GPU_EXPECT:-3060}"
[[ "${gpu_renderer,,}" == *"${gpu_expect,,}"* ]] || fail "Expected GPU substring '$gpu_expect', received: $gpu_renderer. Set MESA_D3D12_DEFAULT_ADAPTER_NAME to the desired Windows adapter."
grep -Eq '^[[:space:]]*Accelerated:[[:space:]]*yes[[:space:]]*$' <<< "$gpu_info" || fail 'The renderer is not hardware accelerated.'

gpu_gl_version="$(awk -F': ' '/^OpenGL core profile version string:/{print $2; exit}' <<< "$gpu_info")"
[[ "$gpu_gl_version" =~ ^([0-9]+)\.([0-9]+) ]] || fail 'glxinfo did not report the OpenGL core profile version.'
gpu_gl_major="${BASH_REMATCH[1]}"
gpu_gl_minor="${BASH_REMATCH[2]}"
(( gpu_gl_major > 4 || (gpu_gl_major == 4 && gpu_gl_minor >= 2) )) || fail "OpenGL 4.2 or newer is required for the ocean shaders; received $gpu_gl_version. Update the Windows driver and WSL; rebuild with current Ubuntu Mesa packages. Do not override the reported GL version."

printf '\nGPU check passed: %s; OpenGL %s.%s.\n' "$gpu_renderer" "$gpu_gl_major" "$gpu_gl_minor"

