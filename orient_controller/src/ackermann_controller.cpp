#include "orient_controller/ackermann_controller.hpp"

namespace orient_controller
{

void AckermannController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    if (use_rotate_to_heading_) {
      use_rotate_to_heading_ = false;
      RCLCPP_WARN(node->get_logger(), "AckermannController does not support rotate to heading, disabling it.");
    }
    // Load parameters specific to AckermannController if needed
    node->declare_parameter(name + ".wheelbase", 1.5);
    node->declare_parameter(name + ".max_steering_angle", 0.698);
    node->declare_parameter(name + ".steering_angle_scaling", 1.0);

    node->get_parameter(name + ".wheelbase", wheelbase_);
    node->get_parameter(name + ".max_steering_angle", max_steering_angle_);
    node->get_parameter(name + ".steering_angle_scaling", steering_angle_scaling_);
}
/**
 * @brief 计算Ackermann车辆的速度指令
 * 
 * @param pose 当前车辆位姿
 * @param speed 当前车辆速度
 * @param goal_checker 目标检查器（未使用）
 * @return geometry_msgs::msg::TwistStamped 控制指令（线速度和转向角）
 */
geometry_msgs::msg::TwistStamped AckermannController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker *)
{
  // 将全局路径转换到机器人坐标系下
  auto transformed_plan = transformGlobalPlan(pose);
  // 计算前视距离
  double lookahead_dist = getLookAheadDistance(speed);

  // 检查是否允许倒车
  if (allow_reversing_) {
    // 检查路径上速度方向变化点（cusp）
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // 如果到cusp的距离小于前视距离，则缩短前视距离
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  // 获取前视点（carrot point）
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  // 发布前视点用于可视化
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  // 计算行驶方向（正向或反向）
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  // 计算前视点到车辆的平方距离
  const double carrot_dist = std::hypot(carrot_pose.pose.position.x,  carrot_pose.pose.position.y);
    
  // 计算曲率（k = 1 / R）
  double curvature = 0.0;
  if (carrot_dist > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / (carrot_dist * carrot_dist);
  }

  double linear_vel = desired_linear_vel_;
  // 应用速度和曲率约束
  applyConstraints(curvature, transformed_plan, linear_vel, sign);

  // 计算转向角
  double steering_angle = 0.0;
  if (fabs(curvature) > 1e-6 && fabs(linear_vel) > 0.01) {
    double arg = wheelbase_ * curvature;
    if (fabs(arg) > 10.0) {  // 防止atan输入过大
        arg = std::copysign(10.0, arg);
    }
    steering_angle = atan(arg);
    if (sign < 0.0) {
      steering_angle = -steering_angle;  // 倒车时转向角取反
    }

    // 限制转向角在最大范围内
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
  }
  // 填充并返回速度指令消息
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = steering_angle;    // 用于角速度  
  cmd_vel.twist.angular.x = steering_angle_scaling_;   // 计算舵轮转向角
  return cmd_vel;
}

}  // namespace orient_controller

PLUGINLIB_EXPORT_CLASS(orient_controller::AckermannController, nav2_core::Controller)