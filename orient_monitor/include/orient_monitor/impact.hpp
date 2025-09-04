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

#ifndef NAV2_COLLISION_MONITOR__IMPACT_HPP_
#define NAV2_COLLISION_MONITOR__IMPACT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_util/robot_utils.hpp"

#include "orient_monitor/source.hpp"

namespace orient_monitor
{

/**
 * @brief Implementation for laser scanner source
 */
class Impact : public Source
{
public:
  Impact(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);
  ~Impact();

  void configure();
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr robot_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr data_sub_;

};  // class Scan

}  // namespace orient_monitor

#endif  // NAV2_COLLISION_MONITOR__SCAN_HPP_
