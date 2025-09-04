
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "orient_interfaces/action/forklift.hpp" // CHANGE

using namespace nav2_behavior_tree;

namespace orient_behavior_tree
{
class ForkliftAction: public BtActionNode<orient_interfaces::action::Forklift>
{
    using ActionT = orient_interfaces::action::Forklift;

public:
    ForkliftAction(
        const std::string & service_node_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf):
        BtActionNode<ActionT>(service_node_name, action_name, conf) {
            
        getInput("action", goal_.action_name);

        int duration;
        getInput("wait_duration", duration);
        if (duration <= 0) {
            RCLCPP_WARN(
            node_->get_logger(), "Wait duration is negative or zero "
            "(%i). Setting to positive.", duration);
            duration *= -1;
        }
        goal_.time.sec = duration;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("action", std::string("dac63004App-up"), "Global frame"),
            BT::InputPort<int>("wait_duration", 1, "Wait time")
        };
    }

    BT::NodeStatus  on_success() {
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
    {
        return std::make_unique<orient_behavior_tree::ForkliftAction>( name, "forklift", config);
    };
    factory.registerBuilder<orient_behavior_tree::ForkliftAction>("Forklift", builder);

}
