#ifndef ORIENT_REFECTOR_SLIDING_WINDOW_FILTER_H
#define ORIENT_REFECTOR_SLIDING_WINDOW_FILTER_H

#include <deque>
#include <cmath>
#include <vector>
#include "rclcpp/time.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


// 位姿数据结构

class SlidingWindowFilter {
    struct Pose {
        geometry_msgs::msg::Pose pose;
        rclcpp::Time timestamp; // 时间戳
    };
    
public:
    SlidingWindowFilter(size_t window_size=5);

    // 更新位姿并返回滤波后结果
    geometry_msgs::msg::Pose update(const rclcpp::Time &timestamp,const geometry_msgs::msg::Pose& new_pose);

    // 设置速度突变阈值（默认2倍）
    void setThreshold(double threshold) { 
        velocity_threshold_ = threshold; 
    }

private:
    std::deque<Pose> pose_window_;
    size_t window_size_;
    double velocity_threshold_ = 2.0;

    // 计算当前瞬时速度
    double calculateCurrentVelocity() const;

    // 计算窗口平均速度
    double calculateAverageVelocity() const;

    // 计算窗口中位位姿
    geometry_msgs::msg::Pose  getMedianPose() const;
};
#endif