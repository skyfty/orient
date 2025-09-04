#ifndef GOAL_CHECKER_H
#define GOAL_CHECKER_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "orient_interfaces/msg/check_point_state_map.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_state.hpp"             // CHANGE
#include "orient_interfaces/srv/get_checkpoint_scout.hpp"             // CHANGE

namespace orient_controller
{

class GoalChecker : public nav2_core::GoalChecker
{
public:
  GoalChecker();
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  bool isCheckpointScoutEmpty();

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  // Cached squared xy_goal_tolerance_
  double xy_goal_tolerance_sq_;
  // Dynamic parameters handler
  std::string plugin_name_;

  
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Logger logger_ {rclcpp::get_logger("GoalChecker")};

  rclcpp::Client<orient_interfaces::srv::GetCheckpointScout>::SharedPtr checkpoint_scout_client_;


};

}
#endif // GOAL_CHECKER_H
