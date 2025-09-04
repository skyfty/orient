// Copyright (c) 2022 Samsung Research
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

#ifndef ORIENT_UTILS_TRANSFORM_HPP_
#define ORIENT_UTILS_TRANSFORM_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace orient_utils
{
  geometry_msgs::msg::Quaternion quaternion_multiply(
      const geometry_msgs::msg::Quaternion& q1,
      const geometry_msgs::msg::Quaternion& q2);

  geometry_msgs::msg::Quaternion quaternion_conjugate(
    const geometry_msgs::msg::Quaternion& q);
  geometry_msgs::msg::Point rotate_vector(
    const geometry_msgs::msg::Quaternion& q,
    const geometry_msgs::msg::Point& v);

   geometry_msgs::msg::Point transform_map_to_baselink(
    const geometry_msgs::msg::Point& map_pose,     // map下的目标点
    const geometry_msgs::msg::Pose& base_in_map);

   geometry_msgs::msg::Pose transform_map_to_baselink(
    const geometry_msgs::msg::Pose& map_point,     // map下的目标点
    const geometry_msgs::msg::Pose& base_in_map);
  geometry_msgs::msg::Pose transform_baselink_to_map(
    const geometry_msgs::msg::Pose& baselink_pose,     // base_link下的目标点
    const geometry_msgs::msg::Pose& base_in_map);
}  

#endif  // ORIENT_UTILS_HPP_
