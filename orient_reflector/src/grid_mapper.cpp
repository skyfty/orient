#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include "orient_reflector/probability_grid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "orient_interfaces/msg/reflector_grid.hpp"     // CHANGE
#include "orient_interfaces/msg/reflector_map.hpp"     // CHANGE

#include <memory>
#include <Eigen/Dense>
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace orient_reflector {

class GridMapperNode : public nav2_util::LifecycleNode {
public:
    GridMapperNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : nav2_util::LifecycleNode("grid_mapper_node", "", options) 
    {
        // 参数声明
        this->declare_parameter("resolution", 0.05);
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("base_link_frame_id", "base_footprint");
        this->declare_parameter("min_laser_range", 0.1);
        this->declare_parameter("max_laser_range", 300.0);
        
    }

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        // 获取参数
        resolution_ = this->get_parameter("resolution").as_double();
        map_frame_id_ = this->get_parameter("map_frame_id").as_string();
        base_link_frame_id_ = this->get_parameter("base_link_frame_id").as_string();
        min_laser_range_ = this->get_parameter("min_laser_range").as_double();
        max_laser_range_ = this->get_parameter("max_laser_range").as_double();
        grid_ = std::make_unique<ProbabilityGrid>(resolution_, 400, 400, -10.0, -10.0);
        
        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建反射器位置订阅器
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        reflector_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "reflector_pose", rclcpp::QoS(10),
             std::bind(&GridMapperNode::reflector_pose_callback, this, _1));

        // 创建激光扫描订阅器
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan/filtered", rclcpp::SensorDataQoS(),
            std::bind(&GridMapperNode::laser_scan_callback, this, _1));
        reflector_map_sub_ = this->create_subscription<orient_interfaces::msg::ReflectorMap>(
            "reflector/map", qos,
            std::bind(&GridMapperNode::reflector_map_callback, this, _1));
        
        // 创建地图发布器
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

        // 创建定时器用于地图发布
        publish_timer_ = this->create_wall_timer(100ms, [this]() { this->publish_map(); });
        
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        // 激活订阅器和定时器
        map_pub_->on_activate();
        active_ = true;
        RCLCPP_INFO(this->get_logger(), "Grid Mapper Node activated");
        // create bond connection
        createBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)  {
        // 停止订阅器和定时器
        map_pub_->on_deactivate();
        active_ = false;
        // destroy bond connection
        destroyBond();
        RCLCPP_INFO(this->get_logger(), "Grid Mapper Node deactivated");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)  {
        // 清理资源
        laser_sub_.reset();
        grid_.reset();
        map_pub_.reset();
        publish_timer_.reset();
        tf_buffer_. reset();
        tf_listener_.reset();

        RCLCPP_INFO(this->get_logger(), "Grid Mapper Node cleaned up");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    
private:
   void reflector_map_callback(const orient_interfaces::msg::ReflectorMap::SharedPtr msg) {
        if (!map_initialized_) {
            // 重置地图，以当前位置为中心
            grid_ = std::make_unique<ProbabilityGrid>(resolution_, grid_->getWidth(), grid_->getHeight(), msg->origin.position.x, msg->origin.position.y);
            map_initialized_ = true;
        }

        reflectors_ = msg->reflectors;
        for(auto &reflector : reflectors_) {
            reflector.point.x += msg->origin.position.x;
            reflector.point.y += msg->origin.position.y;
        }
    }

   void reflector_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // 存储最新位姿
        latest_pose_ = msg;
        pose_initialized_ = true;
    }

    geometry_msgs::msg::Pose convert_to_scanner_link(const std::string &frame_id) {
        tf2::Transform tf_map_to_baselink;
        tf2::fromMsg(latest_pose_->pose.pose, tf_map_to_baselink);
        tf2::Transform tf_map_to_base_link;
        auto scanner_to_base_link = tf_buffer_->lookupTransform(base_link_frame_id_, frame_id, tf2::TimePointZero);
        tf2::fromMsg(scanner_to_base_link.transform, tf_map_to_base_link);
        tf2::Transform base_translation;
        base_translation.mult(tf_map_to_baselink, tf_map_to_base_link);
        geometry_msgs::msg::Pose robot_base_link_pose;
        tf2::toMsg(base_translation, robot_base_link_pose);
        return robot_base_link_pose;
    }

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!active_ || !pose_initialized_ || !map_initialized_ || !grid_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,   "Pose not initialized. Skipping scan.");
            return;
        }

        auto robot_pose = convert_to_scanner_link(msg->header.frame_id);
        // 转换激光数据到地图坐标系
        auto scan_points = transform_scan_to_map(robot_pose,msg);
        
        // // 获取机器人位置
        Eigen::Vector2d robot_position(robot_pose.position.x, robot_pose.position.y);

        // // 更新地图
        grid_->updateFromScan(robot_position, scan_points);

        grid_->reinforceReflectors(reflectors_);
        
        // // 检查是否需要扩展地图
        grid_->expandIfNeeded(robot_position.x(), robot_position.y());
    }

    std::vector<Eigen::Vector2d> transform_scan_to_map(const geometry_msgs::msg::Pose &pose,const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        std::vector<Eigen::Vector2d> map_points;

        // 提取机器人位姿
        double robot_x = pose.position.x;
        double robot_y = pose.position.y;
        
        // 从四元数提取偏航角
        tf2::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 生成角度序列
        size_t num_points = scan->ranges.size();
        double angle = scan->angle_min;

        // 转换每个激光点
        for (size_t i = 0; i < num_points; ++i) {
            double r = scan->ranges[i];

            if (std::isnan(r) || r < min_laser_range_ || r > max_laser_range_) {
                angle += scan->angle_increment;
                continue;
            }
            // 激光点在激光坐标系中的坐标
            double laser_x = r * std::cos(angle);
            double laser_y = r * std::sin(angle);
            
            // 激光坐标系到地图坐标系的转换
            double map_x = robot_x + laser_x * std::cos(yaw) - laser_y * std::sin(yaw);
            double map_y = robot_y + laser_x * std::sin(yaw) + laser_y * std::cos(yaw);
            
            map_points.emplace_back(map_x, map_y);
            angle += scan->angle_increment;
        }
        
        return map_points;
    }

    void publish_map() {
        if (!map_initialized_ || !grid_) {
            return;
        }

        auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        grid_msg->header.stamp = this->now();
        grid_msg->header.frame_id = map_frame_id_;
        grid_msg->info.resolution = grid_->getResolution();
        grid_msg->info.width = grid_->getWidth();
        grid_msg->info.height = grid_->getHeight();
        grid_msg->info.origin.position.x = grid_->getOriginX();
        grid_msg->info.origin.position.y = grid_->getOriginY();
        grid_msg->info.origin.orientation.w = 1.0;
        
        // 复制地图数据
        const auto& grid_data = grid_->getGridData();
        grid_msg->data.assign(grid_data.begin(), grid_data.end());
        
        map_pub_->publish(std::move(grid_msg));
    }

    // ROS2 成员
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reflector_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<orient_interfaces::msg::ReflectorMap>::SharedPtr reflector_map_sub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::atomic<bool> active_{false};
    std::atomic<bool>  pose_initialized_{false};
    std::atomic<bool>  map_initialized_ {false};
    double min_laser_range_;
    double max_laser_range_;
    // 地图构建器
    std::unique_ptr<orient_reflector::ProbabilityGrid> grid_;
    
    // 状态
    std::vector<orient_interfaces::msg::Reflector> reflectors_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;

    double resolution_;
    std::string map_frame_id_;
    std::string base_link_frame_id_ ;
};
} // namespace orient_reflector

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<orient_reflector::GridMapperNode>(options);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}