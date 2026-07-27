#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPositionReset.hh>

namespace spot_gazebo
{
class InitialJointPositionSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure
{
 public:
  void Configure(
      const gz::sim::Entity & entity,
      const std::shared_ptr<const sdf::Element> & sdf,
      gz::sim::EntityComponentManager & ecm,
      gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm) || !sdf->HasElement("joint")) {
      return;
    }

    auto configuration = sdf->Clone();
    auto joint_element = configuration->GetElement("joint");
    while (joint_element) {
      const auto name = joint_element->Get<std::string>("name");
      const auto position = joint_element->Get<double>("position");
      const auto joint = model.JointByName(ecm, name);
      if (joint != gz::sim::kNullEntity) {
        ecm.CreateComponent(
            joint,
            gz::sim::components::JointPositionReset(
                std::vector<double>{position}));
      }
      joint_element = joint_element->GetNextElement("joint");
    }
  }
};
}  // namespace spot_gazebo

IGNITION_ADD_PLUGIN(
    spot_gazebo::InitialJointPositionSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(
    spot_gazebo::InitialJointPositionSystem,
    "spot_gazebo::InitialJointPositionSystem")
