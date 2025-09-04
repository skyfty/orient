
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
namespace orient_behavior_tree
{

class SlowdownController : public BT::DecoratorNode
{
public:
  SlowdownController(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf),
    distance_(1.0),
    first_time_(false)
  {
      getInput("distance", distance_);
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    };
  }

private:
  BT::NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("robot_pose", current_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get robot pose from blackboard");
        return BT::NodeStatus::FAILURE;
    }

    if (status() == BT::NodeStatus::IDLE) {
      start_pose_ = current_pose;
      first_time_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Get euclidean distance
    auto travelled = nav2_util::geometry_utils::euclidean_distance(start_pose_.pose, current_pose.pose);

    if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) || travelled >= distance_) {
      first_time_ = false;
      const BT::NodeStatus child_state = child_node_->executeTick();

      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
          return BT::NodeStatus::FAILURE;
      }
    }

    return status();
  }

  rclcpp::Node::SharedPtr node_;

  geometry_msgs::msg::PoseStamped start_pose_;
  double distance_;
  bool first_time_;
};

}  // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::SlowdownController>("SlowdownController");
}
