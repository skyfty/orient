#include "orient_controller/aline_shim_controller.hpp"
#include "orient_controller/utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace orient_controller
{

AlineShimController::AlineShimController()
: lp_loader_("nav2_core", "nav2_core::Controller"),
  primary_controller_(nullptr)
{
}

void AlineShimController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();

  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  std::string primary_controller;
  double control_frequency;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_disengage_threshold", rclcpp::ParameterValue(0.785));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".forward_sampling_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".primary_controller", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_goal_heading", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".closed_loop", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".angular_disengage_threshold", angular_disengage_threshold_);
  node->get_parameter(plugin_name_ + ".forward_sampling_distance", forward_sampling_distance_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);

  primary_controller = node->get_parameter(plugin_name_ + ".primary_controller").as_string();
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  node->get_parameter(plugin_name_ + ".closed_loop", closed_loop_);

  try {
    primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);
    RCLCPP_INFO(
      logger_, "Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), primary_controller.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
    return;
  }

  primary_controller_->configure(parent, name, tf, costmap_ros);

}

void AlineShimController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "AlineShimController",
    plugin_name_.c_str());

  primary_controller_->activate();
  last_angular_vel_ = std::numeric_limits<double>::max();
}

void AlineShimController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "AlineShimController",
    plugin_name_.c_str());

  primary_controller_->deactivate();
}

void AlineShimController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "AlineShimController",
    plugin_name_.c_str());

  primary_controller_->cleanup();
  primary_controller_.reset();
}

geometry_msgs::msg::TwistStamped AlineShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
    // Rotate to goal heading when in goal xy tolerance
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    // 获取当前机器人朝向
    double pose_yaw = tf2::getYaw(pose.pose.orientation);
    // 获取路径第一个点的朝向
    geometry_msgs::msg::PoseStamped sampled_pt_goal = getSampledPathPt();
    double goal_yaw = tf2::getYaw(sampled_pt_goal.pose.orientation);
    // 计算朝向差
    double angular_distance_to_heading = angles::shortest_angular_distance(pose_yaw, goal_yaw);

    // 判断是否需要旋转
    if (std::abs(angular_distance_to_heading) > angular_disengage_threshold_) {
        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
    }
  
    // If at this point, use the primary controller to path track
    auto cmd_vel = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    
    last_angular_vel_ = cmd_vel.twist.angular.z;
    return cmd_vel;
}

geometry_msgs::msg::PoseStamped AlineShimController::getSampledPathPt()
{
  if (current_path_.poses.size() < 2) {
    throw nav2_core::PlannerException(
            "Path is too short to find a valid sampled path point for rotation.");
  }

  geometry_msgs::msg::Pose start = current_path_.poses.front().pose;
  double dx, dy;

  // Find the first point at least sampling distance away
  for (unsigned int i = 1; i != current_path_.poses.size(); i++) {
    dx = current_path_.poses[i].pose.position.x - start.position.x;
    dy = current_path_.poses[i].pose.position.y - start.position.y;
    if (hypot(dx, dy) >= forward_sampling_distance_) {
      current_path_.poses[i].header.frame_id = current_path_.header.frame_id;
      current_path_.poses[i].header.stamp = clock_->now();  // Get current time transformation
      return current_path_.poses[i];
    }
  }

  throw nav2_core::PlannerException(
          std::string(
            "Unable to find a sampling point at least %0.2f from the robot,"
            "passing off to primary controller plugin.", forward_sampling_distance_));
}

geometry_msgs::msg::TwistStamped
AlineShimController::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  auto current = closed_loop_ ? velocity.angular.z : last_angular_vel_;
  if (current == std::numeric_limits<double>::max()) {
    current = 0.0;
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * rotate_to_heading_angular_vel_;
  const double & dt = control_duration_;
  const double min_feasible_angular_speed = current - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = current + max_angular_accel_ * dt;
  cmd_vel.twist.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  return cmd_vel;
}


void AlineShimController::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  primary_controller_->setPlan(path);
}

void AlineShimController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_rotation_shim_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  orient_controller::AlineShimController,
  nav2_core::Controller)
