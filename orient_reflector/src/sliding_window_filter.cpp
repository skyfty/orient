#include "orient_reflector/sliding_window_filter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/geometry_utils.hpp"


SlidingWindowFilter::SlidingWindowFilter(size_t window_size) 
    : window_size_(window_size) {}

// 更新位姿并返回滤波后结果
geometry_msgs::msg::Pose SlidingWindowFilter::update(const rclcpp::Time &timestamp,const geometry_msgs::msg::Pose& new_pose) {
    // 维护滑动窗口
    if(pose_window_.size() >= window_size_) {
        pose_window_.pop_front();
    }
    SlidingWindowFilter::Pose pose;
    pose.pose = new_pose;
    pose.timestamp = timestamp; // 假设x坐标为时间戳
    pose_window_.push_back(pose);

    // 计算统计量
    if(pose_window_.size() < 2) {
        return new_pose;
    }

    // 计算窗口内速度统计
    const double current_vel = calculateCurrentVelocity();
    const double avg_vel = calculateAverageVelocity();
    const double vel_ratio = current_vel / (avg_vel + 1e-6); // 避免除零

    // 判断速度突变
    if(vel_ratio > velocity_threshold_) {
        // 返回窗口中间值作为稳健估计
        return getMedianPose();
    }
    return new_pose;
}


// 计算当前瞬时速度
double SlidingWindowFilter::calculateCurrentVelocity() const {
    const Pose& prev = pose_window_[pose_window_.size()-2];
    const Pose& curr = pose_window_.back();
    double distance = nav2_util::geometry_utils::euclidean_distance(curr.pose, prev.pose);
    const double dt = (curr.timestamp - prev.timestamp).seconds();
    return distance / dt;
}

// 计算窗口平均速度
double SlidingWindowFilter::calculateAverageVelocity() const {
    double total_vel = 0.0;
    size_t count = 0;
    
    for(size_t i=1; i<pose_window_.size(); ++i) {
        double distance = nav2_util::geometry_utils::euclidean_distance(pose_window_[i].pose, pose_window_[i-1].pose);
        const double dt = (pose_window_[i].timestamp - pose_window_[i-1].timestamp).seconds();
        total_vel += distance / dt;
        ++count;
    }
    return total_vel / count;
}

// 计算窗口中位位姿
geometry_msgs::msg::Pose  SlidingWindowFilter::getMedianPose() const {
    geometry_msgs::msg::Pose median_pose;
    if(pose_window_.empty()) {
        return median_pose;
    }
    
    // 提取x坐标排序找中位数
    std::vector<double> xs, ys, thetas;
    for(const auto& p : pose_window_) {
        xs.push_back(p.pose.position.x);
        ys.push_back(p.pose.position.y);
        thetas.push_back(tf2::getYaw(p.pose.orientation));
    }
    
    auto mid = pose_window_.size()/2;
    std::nth_element(xs.begin(), xs.begin()+mid, xs.end());
    std::nth_element(ys.begin(), ys.begin()+mid, ys.end());
    std::nth_element(thetas.begin(), thetas.begin()+mid, thetas.end());
    median_pose.position.x = xs[mid];
    median_pose.position.y = ys[mid];
    median_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(thetas[mid]);
    median_pose.position.z = 0.0; // Assuming z is not used
    return median_pose;
}