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

#ifndef ORIENT_UTILS_ESTIMATE_HPP_
#define ORIENT_UTILS_ESTIMATE_HPP_

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
#include "orient_interfaces/msg/check_point.hpp"             // CHANGE

namespace orient_utils
{

std::vector<geometry_msgs::msg::Pose> estimate_poses(
    const std::vector<geometry_msgs::msg::Point> &in_points,
    geometry_msgs::msg::Pose last_pose, bool backup = false);

std::vector<geometry_msgs::msg::Pose> estimate_poses( const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &end,double interpolation = 0.1);

std::vector<geometry_msgs::msg::Pose> estimate_poses_backup(const std::vector<geometry_msgs::msg::Point> &in_points,geometry_msgs::msg::Pose last_pose);
std::vector<geometry_msgs::msg::Point> getStraightLinePoints(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & goal, double interpolation_resolution);
std::vector<orient_interfaces::msg::CheckPoint> getStraightLinePoints(const orient_interfaces::msg::CheckPoint & start, const orient_interfaces::msg::CheckPoint & goal, double interpolation_resolution = 1);

}  

#endif  // ORIENT_UTILS_HPP_
