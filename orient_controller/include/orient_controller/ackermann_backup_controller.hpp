#ifndef ORIENT_ACKERMANN_BACKUP_CONTROLLER_HPP_
#define ORIENT_ACKERMANN_BACKUP_CONTROLLER_HPP_
#include "orient_controller/ackermann_controller.hpp"
#include "orient_interfaces/msg/ackermann_backup_path.hpp"      // CHANGE

namespace orient_controller
{
class AckermannBackupController : public AckermannController
{
public:
  AckermannBackupController() = default;
  ~AckermannBackupController() override = default;

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
   * @brief 根据路径曲率计算转向角
   * @param transformed_plan 转换后的路径
   * @return 计算得到的转向角
   */
  double calculateSteeringAngleFromCurvature(const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;
  double calculateOverallCurvature(const nav_msgs::msg::Path& path);

protected:
  double curvature_ = 0.0;  // 曲率符号，默认为正向
  double steering_angle_threshold_ = 1.0; // 转向角阈值，单位为弧度
  double curvature_lookahead_dist_ = 1.0; // 曲率前视距离，单位为米
  std::optional<double> curvature_sign_; // 曲率符号，默认为正向

};

}  // namespace orient_controller
#endif