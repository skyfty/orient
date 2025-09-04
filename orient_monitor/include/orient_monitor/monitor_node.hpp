// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ORIENT_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define ORIENT_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"

#include "nav2_msgs/action/wait.hpp"
#include "orient_monitor/types.hpp"
#include "orient_monitor/circle.hpp"
#include "orient_monitor/source.hpp"
#include "orient_monitor/scan.hpp"
#include "orient_monitor/range.hpp"
#include "orient_monitor/gpio.hpp"
#include "orient_interfaces/msg/collision_action.hpp" // CHANGE
#include "orient_interfaces/msg/agent_state.hpp" // CHANGE

namespace orient_monitor
{

enum State{
  STATE_AGENT_ERROR = 0x1,
  STATE_COLLISION_SLOWDOWN = 0x2,
  STATE_COLLISION_STOP = 0x4,
  STATE_COLLISION = STATE_COLLISION_SLOWDOWN | STATE_COLLISION_STOP,
};

/**
 * @brief Collision Monitor ROS2 node
 */
class Monitor : public nav2_util::LifecycleNode
{
public:
  explicit Monitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Monitor();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void publishVelocity(const Action & robot_action);

  bool getParameters(
    std::string & cmd_vel_in_topic,
    std::string & cmd_vel_out_topic);

  bool configurePolygons(
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);

  bool configureSources(
    const std::string & base_frame_id,
    const std::string & odom_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  void process(const Velocity & cmd_vel_in);

  bool processStopSlowdown(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  bool processApproach(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;


  Action getStopAction();

  void printAction(const Action & robot_action) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  void collide_check();
  void set_state(uint64_t state, bool b);
  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<Source>> sources_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::CollisionAction>::SharedPtr collision_action_publisher_;

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Previous robot action
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  rclcpp::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  rclcpp::Duration stop_pub_timeout_;


  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_heartbeat_time_;
  double timeout_sec_;

  uint64_t state_ = 0;
  double check_frequency_ = 40.0; // Hz
  std::atomic<bool> stop_flag_{false};
  std::thread worker_thread_;
  std::unique_ptr<Gpio> gpio_;
};  // class Monitor

}  // namespace orient_monitor

#endif  // ORIENT_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
