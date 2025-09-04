
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/node_thread.hpp"
#include "orient_govern/govern.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

namespace orient_tests
{
class GovernServerTester;

class GovernTester : public rclcpp::Node
{
public:

  GovernTester(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void activate();

private:
  // A thread for spinning the ROS node
  std::unique_ptr<nav2_util::NodeThread> spin_thread_;
  // The global planner
  rclcpp::Publisher<orient_interfaces::msg::Task>::SharedPtr task_publisher_;
}; 

}