

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "orient_interfaces/srv/is_checkpoint_free.hpp" // CHANGE
#include "orient_interfaces/msg/check_point_state.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace orient_behavior_tree
{
class DistanceCheckPointCondition: public BT::ConditionNode
{
public:
    DistanceCheckPointCondition(
        const std::string & service_node_name,
        const BT::NodeConfiguration & conf):
        BT::ConditionNode(service_node_name, conf),
        distance_(1.0)
    {
        getInput("distance", distance_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus tick() {
        orient_interfaces::msg::CheckPointState checkpoint;
        if (!getInput<orient_interfaces::msg::CheckPointState>("checkpoint", checkpoint)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get checkpoint state from blackboard");
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getInput<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get robot pose from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        auto travelled = nav2_util::geometry_utils::euclidean_distance(robot_pose.pose.position, checkpoint.position);
        return travelled >= distance_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
            BT::InputPort<orient_interfaces::msg::CheckPointState>("checkpoint"),
            BT::InputPort<double>("distance", 1.0, "Distance"),
        };
    }
    rclcpp::Node::SharedPtr node_;
    double distance_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::DistanceCheckPointCondition>("DistanceCheckPoint");
}
