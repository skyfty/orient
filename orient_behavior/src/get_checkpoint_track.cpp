
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "orient_interfaces/msg/check_point_state.hpp"
#include "orient_interfaces/msg/checkpoint_track.hpp"             // CHANGE


namespace orient_behavior_tree
{
class GetCheckPointTrack :public BT::ConditionNode
{
public:
    GetCheckPointTrack(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf):
        BT::ConditionNode(condition_name, conf) {

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
   
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        checkpoint_track_sub_ = node_->create_subscription<orient_interfaces::msg::CheckpointTrack>(
            "checkpoint/track",
            rclcpp::QoS(1),
            [&](const orient_interfaces::msg::CheckpointTrack::SharedPtr msg){
                this->last_ = msg->last;
                this->next_ = msg->next;
                this->remains_ = msg->remains; 
            },
            sub_option);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<orient_interfaces::msg::CheckPointState>("next_checkpoint"),
            BT::OutputPort<orient_interfaces::msg::CheckPointState>("last_checkpoint")
        };
    }

    BT::NodeStatus tick() {
        callback_group_executor_.spin_some();
        setOutput("next_checkpoint", next_);
        setOutput("last_checkpoint", last_);
        config().blackboard->template set<uint32_t>("remains", remains_);
        return BT::NodeStatus::SUCCESS;
    }

    rclcpp::Node::SharedPtr node_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<orient_interfaces::msg::CheckpointTrack>::SharedPtr checkpoint_track_sub_;

    orient_interfaces::msg::CheckPointState last_;
    orient_interfaces::msg::CheckPointState next_;
    uint32_t remains_ = 0;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::GetCheckPointTrack>("GetCheckPointTrack");
}
