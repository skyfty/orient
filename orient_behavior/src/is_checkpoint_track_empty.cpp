
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "orient_interfaces/srv/is_checkpoint_track_empty.hpp"


namespace orient_behavior_tree
{
class IsCheckPointTrackEmptyAction : public BT::ConditionNode
{
public:
    IsCheckPointTrackEmptyAction(
        const std::string & service_node_name,
        const BT::NodeConfiguration & conf):
        BT::ConditionNode(service_node_name, conf) {

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }
    
    IsCheckPointTrackEmptyAction() = delete;
    
    static BT::PortsList providedPorts() {
        return {

        };
    }
    BT::NodeStatus tick() {
        uint32_t remains = config().blackboard->get<uint32_t>("remains");
        return remains == 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    rclcpp::Node::SharedPtr node_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::IsCheckPointTrackEmptyAction>("IsCheckPointTrackEmpty");
}
