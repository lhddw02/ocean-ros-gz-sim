// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "MooringMath.hh"

namespace
{
void CheckNear(double actual, double expected, const char *label)
{
  if (std::abs(actual - expected) > 1e-9)
  {
    std::cerr << label << ": got " << actual << ", expected " << expected
              << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // namespace

int main()
{
  using ocean_station_systems::PlanarMooringParameters;
  using ocean_station_systems::PlanarMooringWrench;
  constexpr double pi = 3.14159265358979323846;
  PlanarMooringParameters parameters;
  parameters.horizontalStiffness = 10.0;
  parameters.horizontalDamping = 2.0;
  parameters.maxHorizontalForce = 50.0;
  parameters.yawStiffness = 4.0;
  parameters.yawDamping = 3.0;
  parameters.maxYawTorque = 20.0;

  const auto restoring = PlanarMooringWrench(1, -2, 3, -4, 0.1, 0.2, parameters);
  CheckNear(restoring[0], -16.0, "restoring force x opposes displacement/velocity");
  CheckNear(restoring[1], 28.0, "restoring force y opposes displacement/velocity");
  CheckNear(restoring[2], -1.0, "yaw damping sign");

  const auto capped = PlanarMooringWrench(30, 40, 0, 0, 1, 20, parameters);
  CheckNear(std::hypot(capped[0], capped[1]), 50.0, "force magnitude cap");
  CheckNear(capped[0], -30.0, "force cap preserves direction x");
  CheckNear(capped[1], -40.0, "force cap preserves direction y");
  CheckNear(capped[2], -20.0, "yaw torque cap");

  const auto crossing = PlanarMooringWrench(0, 0, 0, 0,
      (-pi + 0.01) - (pi - 0.01), 0, parameters);
  CheckNear(crossing[2], -0.08, "short yaw path across minus/plus pi");

  parameters.maxHorizontalForce = 0.0;
  parameters.maxYawTorque = 0.0;
  const auto disabled = PlanarMooringWrench(5, -5, 1, -1, 1, 1, parameters);
  CheckNear(disabled[0], 0.0, "zero force limit x");
  CheckNear(disabled[1], 0.0, "zero force limit y");
  CheckNear(disabled[2], 0.0, "zero yaw torque limit");
  std::cout << "Mooring restoring force, damping, saturation and yaw-wrap checks passed.\n";
}
