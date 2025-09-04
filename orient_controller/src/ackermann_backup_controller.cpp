#include "orient_controller/ackermann_backup_controller.hpp"
#include <Eigen/Dense>  // 用于最小二乘拟合
namespace orient_controller
{

void AckermannBackupController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    AckermannController::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    node->declare_parameter(name + ".steering_angle_threshold", 1.0);
    node->declare_parameter(name + ".curvature_lookahead_dist", 1.5);
    steering_angle_threshold_ = node->get_parameter(name + ".steering_angle_threshold").as_double();
    curvature_lookahead_dist_ = node->get_parameter(name + ".curvature_lookahead_dist").as_double();
}

void AckermannBackupController::cleanup()
{
  AckermannController::cleanup();
}

void AckermannBackupController::activate()
{
 
  AckermannController::activate();
}

void AckermannBackupController::deactivate()
{
  AckermannController::deactivate();
}

// 三点法计算整体曲率（带方向）
double AckermannBackupController::calculateOverallCurvature(const nav_msgs::msg::Path& path) {
    // 确保路径包含足够点
    if (path.poses.size() < 3) {
        return 0.0;  // 直线路径
    }

    // 选取三点：起点、中点和终点
    const size_t start_idx = 0;
    const size_t mid_idx = path.poses.size() / 2;
    const size_t end_idx = path.poses.size() - 1;

    const auto& p1 = path.poses[start_idx].pose.position;  // 起点
    const auto& p2 = path.poses[mid_idx].pose.position;    // 中点
    const auto& p3 = path.poses[end_idx].pose.position;    // 终点

    // 计算向量
    const double dx1 = p2.x - p1.x, dy1 = p2.y - p1.y;  // P1->P2
    const double dx2 = p3.x - p2.x, dy2 = p3.y - p2.y;  // P2->P3
    const double dx3 = p3.x - p1.x, dy3 = p3.y - p1.y;  // P1->P3

    // 计算向量长度
    const double d1 = std::hypot(dx1, dy1);
    const double d2 = std::hypot(dx2, dy2);
    const double chord = std::hypot(dx3, dy3);  // 弦长

    // 计算向量夹角（使用点积公式）
    const double dot = dx1 * dx2 + dy1 * dy2;
    const double cos_theta = dot / (d1 * d2);
    
    // 处理数值误差导致的无效值
    double theta;
    if (cos_theta >= 1.0) {
        theta = 0.0;  // 三点共线
    } else if (cos_theta <= -1.0) {
        theta = M_PI; // 三点成直线但方向相反
    } else {
        theta = std::acos(cos_theta); // 实际角度
    }

    // 计算曲率 (κ = 2 * sin(θ) / chord)
    // 当三点接近共线时，sin(theta)≈theta
    const double sin_theta = (theta < 0.1) ? theta : std::sin(theta);
    return 2.0 * sin_theta / chord;
}
geometry_msgs::msg::TwistStamped AckermannBackupController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goalChecker)
{
  // 将全局路径转换到机器人坐标系下
  auto transformed_plan = transformGlobalPlan(pose);

  // 获取前视点（carrot point）
  auto carrot_pose = getLookAheadPoint(lookahead_dist_, transformed_plan);
  // 发布前视点用于可视化
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  // 计算前视点到车辆的平方距离
  const double carrot_dist = std::hypot(carrot_pose.pose.position.x,  carrot_pose.pose.position.y);

  // 计算行驶方向（正向或反向）
  double sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  double linear_vel = desired_linear_vel_;

  // 应用接近目标点时的速度缩放
  applyApproachVelocityScaling(transformed_plan, linear_vel);

  // 限制线速度在有效范围内
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);

  // 根据sign调整线速度方向（正向或反向）
  linear_vel = sign * linear_vel;

  if (curvature_sign_ == std::nullopt) {
    curvature_sign_ = transformed_plan.poses.back().pose.position.y >= 0.0 ? 1.0 : -1.0;
  }
  // 计算曲率（k = 1 / R）
  double curvature = 0.0;
  if (carrot_dist > curvature_lookahead_dist_) {
    curvature = curvature_sign_.value() * curvature_ * steering_angle_threshold_;
    // curvature = 2.0 * carrot_pose.pose.position.y / (carrot_dist * carrot_dist);

  }

  // 计算转向角
  double steering_angle = 0.0;
  if (fabs(curvature) > 1e-6 && fabs(linear_vel) > 0.01) {
    steering_angle = atan( wheelbase_ * curvature);
    // 限制转向角在最大范围内
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
  }
  // 填充并返回速度指令消息
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = steering_angle;
  cmd_vel.twist.angular.x = steering_angle_scaling_;   // 计算舵轮转向角
  return cmd_vel;
}

void AckermannBackupController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  curvature_ = calculateOverallCurvature(path);
  curvature_sign_ = std::nullopt; // 重置曲率符号

  RCLCPP_INFO(rclcpp::get_logger("AckermannBackupController"), "Average curvature of global_plan_: %f", curvature_);
}

}  // namespace orient_controller

PLUGINLIB_EXPORT_CLASS(orient_controller::AckermannBackupController, nav2_core::Controller)