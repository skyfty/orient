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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "orient_utils/estimate.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace orient_utils
{

std::vector<geometry_msgs::msg::Pose> estimate_poses( const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &end,double interpolation) {
    std::vector<geometry_msgs::msg::Pose>  poses;
    auto points = getStraightLinePoints(start.position, end.position, interpolation);
    poses = estimate_poses(points, start, false);
    return poses;
}

geometry_msgs::msg::Pose estimate_pose(const geometry_msgs::msg::Point &point1,const geometry_msgs::msg::Point &point2) {
    geometry_msgs::msg::Pose pose;
    pose.position = point1;

    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double final_angle = atan2(dy, dx);
    if (std::isnan(final_angle) || std::isinf(final_angle)) {
      final_angle = 0.0;
    }
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(final_angle);
    return pose;
}

std::vector<geometry_msgs::msg::Pose> estimate_poses(const std::vector<geometry_msgs::msg::Point> &in_points,geometry_msgs::msg::Pose last_pose, bool backup) {
  std::vector<geometry_msgs::msg::Pose>  poses;
  if (in_points.empty()) {
    return poses;
  }
  poses.reserve(in_points.size());

  if (backup) {
    for (size_t i = in_points.size() - 1; i > 0; --i) {
      const geometry_msgs::msg::Point &next_pose = in_points[i - 1];
      last_pose = estimate_pose(in_points[i], next_pose);
      poses.push_back(last_pose);
    }
    // 由于是倒序生成，需要反转
    std::reverse(poses.begin(), poses.end());
  } else {
    for (size_t i = 0; i < in_points.size() - 1; ++i) {
      const geometry_msgs::msg::Point &next_pose = in_points[i + 1];
      last_pose = estimate_pose(in_points[i], next_pose);
      poses.push_back(last_pose);
    }
    geometry_msgs::msg::Pose pose;
    pose.position = in_points.back();
    pose.orientation = last_pose.orientation;
    poses.push_back(pose);
  }
  return poses;
}


template<typename T>
std::vector<T> getStraightLinePoints(const T & start, const T & goal, double interpolation_resolution) {
  std::vector<T> points;
  int total_number_of_loop = std::hypot(goal.x - start.x, goal.y - start.y) / interpolation_resolution;
  double x_increment = (goal.x - start.x) / total_number_of_loop;
  double y_increment = (goal.y - start.y) / total_number_of_loop;
  for (int i = 0; i < total_number_of_loop; ++i) {
    T p;
    p.x = start.x + x_increment * i;
    p.y = start.y + y_increment * i;
    p.z = 0.0;
    points.push_back(p);
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> getStraightLinePoints(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & goal, double interpolation_resolution) {
    return getStraightLinePoints<geometry_msgs::msg::Point>(start, goal, interpolation_resolution);
}

}  // namespace orient_utils
