// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include "MooringMath.hh"

namespace ocean_station_systems
{
namespace
{
double FiniteParameter(const std::shared_ptr<const sdf::Element> &sdf,
    const std::string &name, double fallback)
{
  const double value = sdf->Get<double>(name, fallback).first;
  if (!std::isfinite(value))
    throw std::invalid_argument(name + " must be finite");
  return value;
}

double NonNegativeParameter(const std::shared_ptr<const sdf::Element> &sdf,
    const std::string &name, double fallback)
{
  const double value = FiniteParameter(sdf, name, fallback);
  if (value < 0.0)
    throw std::invalid_argument(name + " must be nonnegative");
  return value;
}
}  // namespace

class StationMooring final : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &) override
  {
    this->configured = false;
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm))
    {
      gzerr << "StationMooring must be attached to a model.\n";
      return;
    }

    const std::string linkName =
        sdf->Get<std::string>("link_name", "").first;
    this->link.ResetEntity(linkName.empty() ? model.CanonicalLink(ecm) :
        model.LinkByName(ecm, linkName));
    if (!this->link.Valid(ecm))
    {
      gzerr << "StationMooring: link [" << linkName << "] was not found in ["
            << model.Name(ecm) << "].\n";
      return;
    }

    this->link.EnableVelocityChecks(ecm);
    const auto initialPose = this->link.WorldPose(ecm);
    if (!initialPose)
    {
      gzerr << "StationMooring: initial world pose is unavailable.\n";
      return;
    }

    try
    {
      this->anchorX = FiniteParameter(sdf, "anchor_x", initialPose->Pos().X());
      this->anchorY = FiniteParameter(sdf, "anchor_y", initialPose->Pos().Y());
      this->anchorYaw = FiniteParameter(sdf, "anchor_yaw",
          initialPose->Rot().Yaw());
      this->parameters.horizontalStiffness = NonNegativeParameter(sdf,
          "horizontal_stiffness", this->parameters.horizontalStiffness);
      this->parameters.horizontalDamping = NonNegativeParameter(sdf,
          "horizontal_damping", this->parameters.horizontalDamping);
      this->parameters.maxHorizontalForce = NonNegativeParameter(sdf,
          "max_horizontal_force", this->parameters.maxHorizontalForce);
      this->parameters.yawStiffness = NonNegativeParameter(sdf,
          "yaw_stiffness", this->parameters.yawStiffness);
      this->parameters.yawDamping = NonNegativeParameter(sdf,
          "yaw_damping", this->parameters.yawDamping);
      this->parameters.maxYawTorque = NonNegativeParameter(sdf,
          "max_yaw_torque", this->parameters.maxYawTorque);
      this->rollPitchDamping = NonNegativeParameter(sdf,
          "roll_pitch_damping", this->rollPitchDamping);
      this->maxRollPitchTorque = NonNegativeParameter(sdf,
          "max_roll_pitch_torque", this->maxRollPitchTorque);
    }
    catch (const std::invalid_argument &error)
    {
      gzerr << "StationMooring: " << error.what() << ". Plugin disabled.\n";
      return;
    }

    this->configured = true;
    gzmsg << "StationMooring: [" << model.Name(ecm) << "] anchored at x="
          << this->anchorX << ", y=" << this->anchorY << ", yaw="
          << this->anchorYaw << "; heave remains free.\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->configured || info.paused ||
        info.dt <= std::chrono::steady_clock::duration::zero() ||
        !this->link.Valid(ecm))
      return;

    const auto pose = this->link.WorldPose(ecm);
    const auto velocity = this->link.WorldLinearVelocity(ecm);
    const auto angularVelocity = this->link.WorldAngularVelocity(ecm);
    if (!pose || !velocity || !angularVelocity)
      return;

    const double dx = pose->Pos().X() - this->anchorX;
    const double dy = pose->Pos().Y() - this->anchorY;
    const double yawError = pose->Rot().Yaw() - this->anchorYaw;
    if (!std::isfinite(dx) || !std::isfinite(dy) ||
        !std::isfinite(yawError) || !velocity->IsFinite() ||
        !angularVelocity->IsFinite())
      return;

    const auto wrench = PlanarMooringWrench(dx, dy,
        velocity->X(), velocity->Y(), yawError, angularVelocity->Z(),
        this->parameters);
    double tx = -this->rollPitchDamping * angularVelocity->X();
    double ty = -this->rollPitchDamping * angularVelocity->Y();
    ClampPlanarMagnitude(tx, ty, this->maxRollPitchTorque);

    // Add to other systems' wrenches instead of overriding hydrodynamics.
    // The mooring attachment is the link origin; set it at the hull COM.
    this->link.AddWorldWrench(ecm, gz::math::Vector3d(wrench[0], wrench[1], 0),
        gz::math::Vector3d(tx, ty, wrench[2]));
  }

private:
  gz::sim::Link link;
  PlanarMooringParameters parameters;
  double anchorX{0.0};
  double anchorY{0.0};
  double anchorYaw{0.0};
  double rollPitchDamping{0.0};
  double maxRollPitchTorque{1000.0};
  bool configured{false};
};
}  // namespace ocean_station_systems

GZ_ADD_PLUGIN(ocean_station_systems::StationMooring, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(ocean_station_systems::StationMooring,
    "ocean_station_systems::StationMooring")
