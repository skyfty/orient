
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using namespace nav2_behavior_tree;

namespace orient_behavior_tree
{

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf):
    BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
    {

    }
  void halt() override
  {
      if (should_cancel_goal()) {
        auto future_result = action_client_->async_get_result(goal_handle_);
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to cancel action server for %s", action_name_.c_str());
        }

        on_cancelled();
      }
    }

  void on_wait_for_result(std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>/*feedback*/)
  {
    // Grab the new path
    nav_msgs::msg::Path new_path;
    getInput("path", new_path);

    // Check if it is not same with the current one
    if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
      // the action server on the next loop iteration
      goal_.path = new_path;
      goal_updated_ = true;
    }

    std::string new_controller_id;
    getInput("controller_id", new_controller_id);

    if (goal_.controller_id != new_controller_id) {
      goal_.controller_id = new_controller_id;
      goal_updated_ = true;
    }

    std::string new_goal_checker_id;
    getInput("goal_checker_id", new_goal_checker_id);

    if (goal_.goal_checker_id != new_goal_checker_id) {
      goal_.goal_checker_id = new_goal_checker_id;
      goal_updated_ = true;
    }
  }

    void on_tick() override
    {
        getInput("path", goal_.path);
        getInput("controller_id", goal_.controller_id);
        getInput("goal_checker_id", goal_.goal_checker_id);
    }


  static BT::PortsList providedPorts()
  {
      return providedBasicPorts(
      {
          BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
          BT::InputPort<std::string>("controller_id", ""),
          BT::InputPort<std::string>("goal_checker_id", ""),
      });
  }
};
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<orient_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<orient_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
