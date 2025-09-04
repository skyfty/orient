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

#include "orient_utils/transform.hpp"

namespace orient_utils
{

// 四元数乘法 (q1 * q2)
geometry_msgs::msg::Quaternion quaternion_multiply(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q2)
{
  geometry_msgs::msg::Quaternion result;
  result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
  return result;
}

// 四元数共轭 (inverse for unit quaternion)
geometry_msgs::msg::Quaternion quaternion_conjugate(
    const geometry_msgs::msg::Quaternion& q)
{
  geometry_msgs::msg::Quaternion conj;
  conj.w = q.w;
  conj.x = -q.x;
  conj.y = -q.y;
  conj.z = -q.z;
  return conj;
}

// 用四元数旋转向量
geometry_msgs::msg::Point rotate_vector(
    const geometry_msgs::msg::Quaternion& q,
    const geometry_msgs::msg::Point& v)
{
  // 将向量转为四元数 (实部为0)
  geometry_msgs::msg::Quaternion vec_quat;
  vec_quat.w = 0.0;
  vec_quat.x = v.x;
  vec_quat.y = v.y;
  vec_quat.z = v.z;

  // 旋转计算: q * v * q_conj
  geometry_msgs::msg::Quaternion q_conj = quaternion_conjugate(q);
  geometry_msgs::msg::Quaternion rotated = quaternion_multiply(
      quaternion_multiply(q, vec_quat), q_conj);

  // 返回旋转后的向量
  geometry_msgs::msg::Point result;
  result.x = rotated.x;
  result.y = rotated.y;
  result.z = rotated.z;
  return result;
}

geometry_msgs::msg::Point transform_map_to_baselink(
  const geometry_msgs::msg::Point& map_point,
  const geometry_msgs::msg::Pose& base_in_map) {
      // Step 1: 平移补偿 (map点 - base_link原点)
  geometry_msgs::msg::Point translated;
  translated.x = map_point.x - base_in_map.position.x;
  translated.y = map_point.y - base_in_map.position.y;
  translated.z = map_point.z - base_in_map.position.z;

  // Step 2: 旋转补偿 (用base_link方向的共轭旋转)
  geometry_msgs::msg::Point rotated = rotate_vector(quaternion_conjugate(base_in_map.orientation), translated);
  return rotated;

}
// 主转换函数：map坐标系点 -> base_link坐标系点
geometry_msgs::msg::Pose transform_map_to_baselink(
    const geometry_msgs::msg::Pose& map_pose,     // map下的目标点
    const geometry_msgs::msg::Pose& base_in_map)  // base_link在map中的位姿
{
  // 构造结果Pose (方向需额外旋转)
  geometry_msgs::msg::Pose result;
  result.position = transform_map_to_baselink(map_pose.position, base_in_map);

  // 方向转换: q_result = q_base_conj * q_map
  result.orientation = quaternion_multiply(quaternion_conjugate(base_in_map.orientation),map_pose.orientation);
  
  return result;
}
geometry_msgs::msg::Pose transform_baselink_to_map(
    const geometry_msgs::msg::Pose& baselink_pose,     // base_link下的目标点
    const geometry_msgs::msg::Pose& base_in_map)  // base_link在map中的位姿
{
  // Step 1: 平移补偿 (base_link点 + base_link在map中的位姿)
  geometry_msgs::msg::Point translated;
  translated.x = baselink_pose.position.x + base_in_map.position.x;
  translated.y = baselink_pose.position.y + base_in_map.position.y;
  translated.z = baselink_pose.position.z + base_in_map.position.z;

  // Step 2: 旋转补偿 (用base_link方向的旋转)
  geometry_msgs::msg::Point rotated = rotate_vector(
      base_in_map.orientation, translated);

  // 构造结果Pose (方向需额外旋转)
  geometry_msgs::msg::Pose result;
  result.position = rotated;

  // 方向转换: q_result = q_base * q_baselink
  result.orientation = quaternion_multiply(
      base_in_map.orientation,
      baselink_pose.orientation);

  return result;
}
}  // namespace orient_utils
