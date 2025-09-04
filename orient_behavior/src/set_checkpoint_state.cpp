

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "orient_interfaces/srv/set_checkpoint_state.hpp"
#include "orient_interfaces/msg/check_point_state.hpp"


namespace orient_behavior_tree
{
class SetCheckPointStateAction : public BT::ConditionNode
{

public:
    SetCheckPointStateAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf):
        BT::ConditionNode(condition_name, conf) {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<orient_interfaces::srv::SetCheckpointState>("set_checkpoint_state");
        getInput("state", state_);        
        orient_clientid_ = config().blackboard->template get<std::string>("orient_clientid");

    }

    BT::NodeStatus tick() {
        orient_interfaces::msg::CheckPointState checkpoint;
        if (!getInput<orient_interfaces::msg::CheckPointState>("checkpoint", checkpoint)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get checkpoint state from blackboard");
            return BT::NodeStatus::FAILURE;
        }
        if ((state_ && checkpoint.clientid == orient_clientid_) || (!state_ && checkpoint.clientid != orient_clientid_)) {
            return  BT::NodeStatus::SUCCESS;
        }
        auto request = std::make_shared<orient_interfaces::srv::SetCheckpointState::Request>();
        request->state = state_;
        request->checkpoint = checkpoint.point;
        client_->async_send_request(request);
        return  BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("state"),
            BT::InputPort<orient_interfaces::msg::CheckPointState>("checkpoint")
        };
    }
    rclcpp::Node::SharedPtr node_;
    std::string orient_clientid_;
    rclcpp::Client<orient_interfaces::srv::SetCheckpointState>::SharedPtr client_;
    bool state_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::SetCheckPointStateAction>("SetCheckPointState");
}
