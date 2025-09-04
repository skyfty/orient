

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "orient_interfaces/srv/is_checkpoint_free.hpp" // CHANGE
#include "orient_interfaces/msg/check_point_state.hpp"

namespace orient_behavior_tree
{
class IsCheckPointFreeAction: public BT::ConditionNode
{
public:
    IsCheckPointFreeAction(
        const std::string & service_node_name,
        const BT::NodeConfiguration & conf):
        BT::ConditionNode(service_node_name, conf) {

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        orient_clientid_ = config().blackboard->template get<std::string>("orient_clientid");
    }

    BT::NodeStatus tick() {
        orient_interfaces::msg::CheckPointState checkpoint;
        // Get the checkpoint state from the blackboard
        if (!getInput<orient_interfaces::msg::CheckPointState>("checkpoint", checkpoint)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get checkpoint state from blackboard");
            return BT::NodeStatus::FAILURE;
        }
        return checkpoint.clientid.empty()? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<orient_interfaces::msg::CheckPointState>("checkpoint")
        };
    }

    rclcpp::Node::SharedPtr node_;
    std::string orient_clientid_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::IsCheckPointFreeAction>("IsCheckPointFree");
}
