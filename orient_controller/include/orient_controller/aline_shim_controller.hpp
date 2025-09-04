
#ifndef ORIENT_ALINE_SHIM_CONTROLLER__ORIENT_ROTATION_SHIM_CONTROLLER_HPP_
#define ORIENT_ALINE_SHIM_CONTROLLER__ORIENT_ROTATION_SHIM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "orient_controller/position_goal_checker.hpp"
#include "angles/angles.h"


namespace orient_controller 
{

/**
 * @class nav2_rotation_shim_controller::AlineShimController
 * @brief Rotate to rough path heading controller shim plugin
 */
class AlineShimController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_rotation_shim_controller::AlineShimController
   */
  AlineShimController();

  /**
   * @brief Destrructor for nav2_rotation_shim_controller::AlineShimController
   */
  ~AlineShimController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  geometry_msgs::msg::PoseStamped getSampledPathPt();

  /**
   * @brief Rotates the robot to the rough heading
   * @param angular_distance Angular distance to the goal remaining
   * @param pose Starting pose of robot
   * @param velocity Starting velocity of robot
   * @return Twist command for rotation to rough heading
   */
  geometry_msgs::msg::TwistStamped computeRotateToHeadingCommand(
    const double & angular_distance,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);


  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("AlineShimController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  nav2_core::Controller::Ptr primary_controller_;
  nav_msgs::msg::Path current_path_;
  double forward_sampling_distance_, angular_dist_threshold_, angular_disengage_threshold_;
  double rotate_to_heading_angular_vel_, max_angular_accel_;
  double control_duration_, simulate_ahead_time_;
  bool closed_loop_;
  double last_angular_vel_ = std::numeric_limits<double>::max();

  // Dynamic parameters handler
  std::mutex mutex_;
};
}
#endif