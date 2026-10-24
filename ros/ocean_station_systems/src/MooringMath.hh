// SPDX-License-Identifier: Apache-2.0
#ifndef OCEAN_STATION_SYSTEMS_MOORING_MATH_HH_
#define OCEAN_STATION_SYSTEMS_MOORING_MATH_HH_

#include <algorithm>
#include <array>
#include <cmath>

namespace ocean_station_systems
{
struct PlanarMooringParameters
{
  double horizontalStiffness{600.0};
  double horizontalDamping{900.0};
  double maxHorizontalForce{5000.0};
  double yawStiffness{200.0};
  double yawDamping{300.0};
  double maxYawTorque{1000.0};
};

inline double WrappedAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Limit magnitude, preserving the intended restoring-force direction.
inline void ClampPlanarMagnitude(double &x, double &y, double limit)
{
  const double magnitude = std::hypot(x, y);
  if (magnitude > limit)
  {
    const double scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

// The inputs are displacement from the anchor and WORLD-frame velocities.
// Return world Fx, Fy and Tz. There is no heave force or tilt spring.
inline std::array<double, 3> PlanarMooringWrench(
    double dx, double dy, double vx, double vy,
    double yawError, double angularVelocityZ,
    const PlanarMooringParameters &parameters)
{
  double fx = -parameters.horizontalStiffness * dx -
      parameters.horizontalDamping * vx;
  double fy = -parameters.horizontalStiffness * dy -
      parameters.horizontalDamping * vy;
  ClampPlanarMagnitude(fx, fy, parameters.maxHorizontalForce);
  const double tz = std::clamp(
      -parameters.yawStiffness * WrappedAngle(yawError) -
      parameters.yawDamping * angularVelocityZ,
      -parameters.maxYawTorque, parameters.maxYawTorque);
  return {fx, fy, tz};
}
}  // namespace ocean_station_systems

#endif
