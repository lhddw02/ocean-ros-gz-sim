/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2026 Ocean simulation maintainers
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Adapted from Gazebo Sim's Apache-2.0 Contact system:
// https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc
// A reconstructed native Contact system only scans EachNew<ContactSensor>.
// World reset clears that flag before its next PreUpdate, losing all sensors.
// This system scans all sensors initially / on Reset and restores the runtime
// ContactSensorData components which ECM reset removes from the collisions.

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

namespace ocean_station_systems
{
namespace
{
struct ContactPublisher
{
  std::vector<gz::sim::Entity> collisions;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  gz::msgs::Contacts pending;
};
}  // namespace

class ReplayContact final : public gz::sim::System,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemPostUpdate,
                            public gz::sim::ISystemReset
{
public:
  void PreUpdate(const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &ecm) override
  {
    this->CreateSensors(ecm, this->firstUpdate);
    this->firstUpdate = false;
  }

  void Reset(const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &ecm) override
  {
    // ResetTo restores initial entities but removes their runtime contact data.
    // Rebuild the publishers and their collision list from the restored ECM.
    // No contact or timestamp from the previous simulation epoch survives.
    this->sensors.clear();
    this->CreateSensors(ecm, true);
    this->firstUpdate = false;
    gzmsg << "ReplayContact: restored " << this->sensors.size()
          << " contact sensors after world reset.\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (!info.paused)
    {
      const auto stamp = gz::sim::convert<gz::msgs::Time>(info.simTime);
      for (auto &entry : this->sensors)
      {
        auto &sensor = *entry.second;
        for (const auto collision : sensor.collisions)
        {
          const auto *data =
              ecm.Component<gz::sim::components::ContactSensorData>(collision);
          if (!data)
            continue;
          for (const auto &contact : data->Data().contact())
          {
            auto *copy = sensor.pending.add_contact();
            copy->CopyFrom(contact);
            copy->mutable_header()->mutable_stamp()->CopyFrom(stamp);
          }
        }
        // Match native Contact: only positive contacts are published.
        if (sensor.pending.contact_size() > 0)
        {
          sensor.pending.mutable_header()->mutable_stamp()->CopyFrom(stamp);
          sensor.publisher.Publish(sensor.pending);
        }
        sensor.pending.Clear();
      }
    }

    ecm.EachRemoved<gz::sim::components::ContactSensor>(
        [&](const gz::sim::Entity &entity,
            const gz::sim::components::ContactSensor *) -> bool
        {
          this->sensors.erase(entity);
          return true;
        });
  }

private:
  void CreateSensors(gz::sim::EntityComponentManager &ecm, bool all)
  {
    const auto create = [&](const gz::sim::Entity &entity,
        const gz::sim::components::ContactSensor *contact) -> bool
    {
      if (this->sensors.find(entity) != this->sensors.end())
        return true;
      const auto *parent =
          ecm.Component<gz::sim::components::ParentEntity>(entity);
      if (!parent || !ecm.Component<gz::sim::components::Link>(parent->Data()))
        return true;

      auto sensor = std::make_unique<ContactPublisher>();
      auto contactElement = contact->Data()->GetElement("contact");
      auto collisionElement = contactElement->GetElement("collision");
      for (; collisionElement;
          collisionElement = collisionElement->GetNextElement("collision"))
      {
        const auto collisionName = collisionElement->Get<std::string>();
        const auto collisions = ecm.ChildrenByComponents(parent->Data(),
            gz::sim::components::Collision(),
            gz::sim::components::Name(collisionName));
        if (collisions.empty())
          continue;
        const auto collision = collisions.front();
        sensor->collisions.push_back(collision);
        auto *data =
            ecm.Component<gz::sim::components::ContactSensorData>(collision);
        if (data)
          data->Data().Clear();
        else
          ecm.CreateComponent(collision,
              gz::sim::components::ContactSensorData());
      }

      // Keep Harmonic's exact scoped default and its nested <contact><topic>
      // override; the outer sensor <topic> is intentionally not consulted.
      auto topic = contactElement->Get<std::string>(
          "topic", "__default_topic__").first;
      if (topic == "__default_topic__")
        topic = gz::sim::scopedName(entity, ecm, "/") + "/contact";
      sensor->publisher = sensor->node.Advertise<gz::msgs::Contacts>(topic);
      gzmsg << "ReplayContact publishing on " << topic << '\n';
      this->sensors.emplace(entity, std::move(sensor));
      return true;
    };

    if (all)
      ecm.Each<gz::sim::components::ContactSensor>(create);
    else
      ecm.EachNew<gz::sim::components::ContactSensor>(create);
  }

  bool firstUpdate{true};
  std::unordered_map<gz::sim::Entity,
      std::unique_ptr<ContactPublisher>> sensors;
};
}  // namespace ocean_station_systems

GZ_ADD_PLUGIN(ocean_station_systems::ReplayContact, gz::sim::System,
    gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate,
    gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(ocean_station_systems::ReplayContact,
    "ocean_station_systems::ReplayContact")
