
#ifndef ORIENT_REFLECTOR__LOCALIZATION_HPP_
#define ORIENT_REFLECTOR__LOCALIZATION_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/geometry_utils.hpp"

#include <cstdio>
#include <string>
#include <fstream>
#include <optional>
#include <vector>
#include <queue>
#include <algorithm>
#include <boost/smart_ptr.hpp>
#include <list>

#include "Eigen/Dense"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/robot_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <tf2/utils.h>
#include "yaml-cpp/yaml.h"
#include "nav2_util/occ_grid_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "orient_interfaces/srv/load_reflector_map.hpp"     // CHANGE
#include "orient_interfaces/srv/get_reflector_map.hpp"     // CHANGE
#include "orient_interfaces/srv/get_initial_pose.hpp"     // CHANGE
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "orient_interfaces/msg/reflector_grid.hpp"     // CHANGE
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "orient_reflector/utils.hpp"

using namespace std::placeholders;
using namespace orient_interfaces::msg;
using namespace orient_interfaces::srv;
using namespace nav2_util::geometry_utils;

namespace orient_reflector
{

struct PointTrack {
    pcl::PointXYZINormal position;
    uint32_t observed_frames = 0;
    uint32_t missed_frames = 0;
    rclcpp::Time last_observed;
};
struct Matched {
    Reflector local;
    Reflector global;
};

struct MatchResult {
    double cost{};
    std::vector<Matched> matched;
};

struct Net {
    size_t first{};
    size_t second{};

    double dist{};
    double angle{};

    bool is_similar(const Net &net, double dist_thresh, double /*angle_thresh*/) const {
        return std::fabs(dist - net.dist) < dist_thresh ;
    }
};

struct TrackedReflector {
    Reflector reflector;
    uint32_t observed_frames = 0;
    uint32_t missed_frames = 0;
    rclcpp::Time last_observed;
};

class Localization : public nav2_util::LifecycleNode {
public:
    Localization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) ;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) ;

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) ;

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) ;
    
    void get_reflector_map_callback(const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<GetReflectorMap::Request>,
        std::shared_ptr<GetReflectorMap::Response> response) ;

    void load_reflector_map_callback(const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<LoadReflectorMap::Request> request,
        std::shared_ptr<LoadReflectorMap::Response> response) ;

    std::unique_ptr<orient_interfaces::msg::ReflectorMap> get_reflector_map();
    bool load_reflector_map(const orient_interfaces::msg::ReflectorMap &reflectorMap) ;

    bool load_reflector_map_from_yaml(const std::string &yaml_file );

    void publish_tf(const geometry_msgs::msg::Pose &robot_pose, const rclcpp::Time &stamp);

    void publish_reflector() ;

    void publish_map();

    void publish_network() ;

    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)  ;

    //比较器 最优匹配结果排序
    static bool compare_match_result(MatchResult &m1, MatchResult &m2);


        /**
     * 找出最靠近point的全局反光柱near
     * @param point
     * @param near
     * @return 距离
     */
    double nearest(const geometry_msgs::msg::Point &point, Reflector &near) const ;

   /**
     * 获取局部反光柱经过旋转平移变换后的匹配得分
     * @param reflectors_local
     * @param rotate 旋转
     * @param tran 平移
     * @return
     */
    MatchResult get_matching_score(const std::vector<Reflector> &reflectors_local, double yaw, const geometry_msgs::msg::Point &tran) ;

    
    inline geometry_msgs::msg::Point rotate(const geometry_msgs::msg::Point &point, double yaw) ;
    
    inline double get_theta(const geometry_msgs::msg::Point &p);
    /**
     * 这段代码的主要功能是：
        1. 计算局部和全局反射器之间的旋转角度和平移向量。
        2. 根据这些参数计算匹配分数。
        3. 筛选出符合条件的匹配结果，并存储到 `results` 容器中。
     * @param reflectors_local 局部反光柱集合
     * @param result
     * @return
     */
    bool reflector_matching(const std::vector<Reflector> &reflectors_local, MatchResult &matched) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convert_point_cloud(const std::vector<Reflector> &in_reflectors) ;
    pcl::PointXYZ convert_point(const Reflector &reflector) ;
    geometry_msgs::msg::Point registration(
        const geometry_msgs::msg::Pose &robot_pose, const geometry_msgs::msg::Point &in_point) const ;
    void verify_reflector_tracks(const geometry_msgs::msg::Pose &robot_pose, const std::vector<Reflector> &in_reflectors) ;

    void cleanup_old_reflector() ;

    bool confirm_reflectors() ;

    bool is_accelerated(const geometry_msgs::msg::Twist &, const geometry_msgs::msg::Accel &accel) ;
    void stamp_reflectors(const geometry_msgs::msg::Pose &robot_pose,const std::vector<Reflector> &in_reflectors) ;

        
    geometry_msgs::msg::Pose locate_robot_pose(const std::vector<Matched> &matcheds);
    geometry_msgs::msg::Pose trilateration_from_matched(const std::vector<Matched> &matched);
    void reflector_recognize(const std::vector<Reflector> &reflectors, const std::string &frame_id, const rclcpp::Time &recognize_stamp)  ;
    geometry_msgs::msg::Pose convert_to_base_link(const geometry_msgs::msg::Pose& robot_pose, const std::string &frame_id) ;

    void publish_matched(const std::vector<Matched> &matcheds, const rclcpp::Time &stamp);
    static geometry_msgs::msg::Transform to_geometry_msg_transform(const geometry_msgs::msg::Pose& pose) ;
    geometry_msgs::msg::Pose calculate_odometry_pose( const geometry_msgs::msg::Pose& initial_pose,  const geometry_msgs::msg::Pose& current_pose)  const;
    geometry_msgs::msg::PoseWithCovarianceStamped convert_pose_to_pose(const geometry_msgs::msg::Pose &robot_pose,const rclcpp::Time &stamp) const ;

    void publish_pose(const geometry_msgs::msg::Pose &robot,const rclcpp::Time &stamp)  const;

private:
    void rebuild_global_nets();
    bool global_reflector_existed(const Reflector &reflector) ;
    bool nearest_global_reflector(const Reflector &reflector, int &near, float &dist) ;
    std::vector<Net>  make_reflector_net(const std::vector<Reflector> &reflectors) ;
    geometry_msgs::msg::Pose calculate_pose(const geometry_msgs::msg::Pose &p1) ;
    geometry_msgs::msg::Twist calculate_twist(const geometry_msgs::msg::Pose &p1,const geometry_msgs::msg::Pose &p2, double delta_time) ;
    geometry_msgs::msg::Twist calculate_twist(const geometry_msgs::msg::Pose &robot_pose, double delta_time) ;
    geometry_msgs::msg::Accel calculate_accel(const geometry_msgs::msg::Twist &t1,const geometry_msgs::msg::Twist &t2, double delta_time) ;
    geometry_msgs::msg::Accel calculate_accel(const geometry_msgs::msg::Twist &t1, double delta_time);
    void compute_velocity(const geometry_msgs::msg::Pose &robot_pose, const rclcpp::Time &stamp);

    float compute_cluster_diameter_convex_hull(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster);

    float compute_intensity_std(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) ;

    std::vector<Reflector> filter(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &reflector_cloud) ;
    /*
    反射柱点云检测
    1. 过滤掉小于最小直径的点
    2. 过滤掉强度标准差大于设定值的点
    3. 过滤掉大于最大直径的点
    4. 过滤掉小于最小聚类数的点
    5. 欧式聚类
    */
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr detect_reflectors_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_scan_to_point_cloud(const sensor_msgs::msg::LaserScan::SharedPtr scan) ;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void cleanup_old_tracks();
private:
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;
    bool publish_tf_;
    bool publish_odom_tf_;
    bool marking_;
    double marking_delta_time_ = 0.1;
    double publish_period_sec_ = 1.0;
    bool publish_map_ = true;

    float linear_accel_thresh_ = 1.5f;
    float angular_accel_thresh_ = 0.1f;

    double transform_timeout_;
    geometry_msgs::msg::Pose origin_;

    double tolerance_; // 反光柱容差
    int min_cluster_; // 最小聚类数
    int max_cluster_; // 最大聚类数
    double min_diameter_; // 最小直径
    double max_diameter_; // 最大直径
    double min_intensity_; // 强度标准差
    double max_intensity_; // 强度标准差
    uint32_t min_confirm_frames_ = 3;  // 最小确认帧数
    bool detect_logs_ = false; // 是否打印检测日志
    std::atomic<bool> origin_available_{false};
    std::atomic<bool> map_available_{false};
    std::list<PointTrack> tracks_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    rclcpp_lifecycle::LifecyclePublisher<ReflectorMap>::SharedPtr pub_reflector_map_;
    rclcpp_lifecycle::LifecyclePublisher<ReflectorGrid>::SharedPtr pub_reflector_network_;

    rclcpp::Service<LoadReflectorMap>::SharedPtr load_map_service_;
    rclcpp::Service<GetReflectorMap>::SharedPtr get_map_service_;


    uint32_t pose_window_size_ = 3;
    std::vector<geometry_msgs::msg::Pose> pose_window_;
    geometry_msgs::msg::PoseStamped robot_pose_;

    uint32_t twist_window_size_ = 3;
    std::vector<geometry_msgs::msg::Twist> twists_window_;
    geometry_msgs::msg::TwistStamped twist_;

    uint32_t accel_window_size_ = 3;
    std::vector<geometry_msgs::msg::Accel> accels_window_;
    geometry_msgs::msg::AccelStamped accel_;


    std::vector<Reflector> reflectors_;
    std::vector<Net> nets_global_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_reflectors_;
    std::list<TrackedReflector> reflector_tracks_;

    uint32_t min_observed_frames_ = 3;  // 最小确认帧数
    uint32_t max_miss_frames_ = 5;     // 最大丢失帧数
    float match_distance_thresh_ = 0.3f; // 匹配距离阈值(m)
    double max_age_seconds_ = 2.0; // 轨迹最大年龄(秒)
    double net_match_distance_thresh_ = 0.05f; // 网络匹配距离阈值(m)
    double net_match_angle_thresh_ = 0.1f; // 网络匹配角度阈值(弧度)
    double position_delta_ = 0.05; // 反光柱抖动阈值(m)
    double angle_delta_ = 0.1; // 反光柱抖动阈值(弧度)

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp_lifecycle::LifecyclePublisher<ReflectorGrid>::SharedPtr pub_reflector_matched_;
    rclcpp::Subscription<ReflectorGrid>::SharedPtr reflector_network_subscription_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp_lifecycle::LifecyclePublisher<ReflectorGrid>::SharedPtr pub_reflector_grid_;

        // Since the sensor data from gazebo or the robot is not lifecycle enabled, we won't
    // respond until we're in the active state
    std::atomic<bool> active_{false};

    std::recursive_mutex mutex_;

    // Dedicated callback group and executor for services and subscriptions in AmclNode,
    // in order to isolate TF timer used in message filter.
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::unique_ptr<nav2_util::NodeThread> executor_thread_;
};
}
#endif