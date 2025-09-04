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

#include "orient_monitor/impact.hpp"

#include <cmath>
#include <functional>

namespace orient_monitor
{

Impact::Impact(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Impact", source_name_.c_str());
}

Impact::~Impact()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Impact", source_name_.c_str());
  data_sub_.reset();
}

void Impact::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no own parameters
  getCommonParameters(source_topic);

  data_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    source_topic, rclcpp::QoS(1),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
      robot_pose_ = msg;
    });
}

void Impact::getData(
  const rclcpp::Time &curr_time,
  std::vector<Point> &) const
{
  if (robot_pose_ == nullptr || !sourceValid(robot_pose_->header.stamp, curr_time)) {
      return;
  }
  // data.push_back({robot_pose_->pose.position.x, robot_pose_->pose.position.y});

}


}  // namespace orient_monitor
