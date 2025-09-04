#ifndef ORIENT_ACKERMANN_CONTROLLER_HPP_
#define ORIENT_ACKERMANN_CONTROLLER_HPP_
#include "orient_controller/regulated_pure_pursuit_controller.hpp"

namespace orient_controller
{
class AckermannController : public RegulatedPurePursuitController
{
public:
  AckermannController() = default;
  ~AckermannController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

protected:
    double wheelbase_ = 0.5;  // Example wheelbase length in meters
    double max_steering_angle_ = 2.094;  // Example maximum steering angle in radians
    double steering_angle_scaling_ = 20.0;  // Example steering angle scaling factor
};

}  // namespace orient_controller
#endif