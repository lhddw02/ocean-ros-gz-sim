#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: set-waves.sh WIND_SPEED_M_S [WIND_ANGLE_DEG=135] [STEEPNESS=1]"
  echo "Updates the live FFT ocean wind and steepness (angle in degrees)."
  exit 0
fi

if [[ "$#" -lt 1 || "$#" -gt 3 ]]; then
  echo "Usage: set-waves.sh WIND_SPEED_M_S [WIND_ANGLE_DEG=135] [STEEPNESS=1]" >&2
  exit 2
fi

speed="$1"
angle="${2:-135}"
steepness="${3:-1}"

# Validate and canonicalize numbers before formatting a Gazebo protobuf message.
canonical="$(python3 - "$speed" "$angle" "$steepness" <<'PY'
import math
import sys
values = [float(value) for value in sys.argv[1:]]
if not all(math.isfinite(value) for value in values):
    raise SystemExit('All wave parameters must be finite numbers')
if not 0 < values[0] <= 30:
    raise SystemExit('Wind speed must be greater than 0 and at most 30 m/s')
if not -360 <= values[1] <= 360:
    raise SystemExit('Wind angle must be between -360 and 360 degrees')
if not 0 <= values[2] <= 2:
    raise SystemExit('Steepness must be between 0 and 2')
print(' '.join(format(value, '.17g') for value in values))
PY
 )"
read -r speed angle steepness <<< "$canonical"

# Upstream uses wind_angle (degrees) on transport, wind_angle_deg in SDF.
message="params { key: \"wind_speed\" value { type: DOUBLE double_value: $speed } }
params { key: \"wind_angle\" value { type: DOUBLE double_value: $angle } }
params { key: \"steepness\" value { type: DOUBLE double_value: $steepness } }"

exec gz topic -t /world/ocean/waves -m gz.msgs.Param -p "$message"
