
#include "orient_reflector/localization.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "cartographer_ros/urdf_reader.h"

// #include <ceres/ceres.h>

namespace orient_reflector
{
Localization::Localization(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("orient_localization_node", "", options) {
    
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("odom_frame", std::string("odom"));
    declare_parameter("base_link_frame", std::string("base_footprint"));
    declare_parameter("tolerance", 0.5);
    declare_parameter("min_cluster", 1);
    declare_parameter("max_cluster", 50);
    declare_parameter("min_diameter", 0.03);    //# 最小直径
    declare_parameter("max_diameter", 0.20);    //最大直径
    declare_parameter("min_intensity",  300.0);  //强度标准差
    declare_parameter("max_intensity",  300.0);  //强度标准差
    declare_parameter("position_delta", 0.01);  // 反光柱抖动阈值(m)
    declare_parameter("angle_delta", 0.1); // 反光柱抖动阈值(m)
    declare_parameter("match_distance_thresh", 0.3);        
    declare_parameter("publish_tf", true);
    declare_parameter("publish_odom_tf", true);
    declare_parameter("publish_map", true);
    declare_parameter("transform_tolerance",rclcpp::ParameterValue(0.1));
    declare_parameter("marking", false);
    declare_parameter("marking_delta_time", 0.5);
    declare_parameter("reflector_filename", "");
    declare_parameter("publish_period_sec", rclcpp::ParameterValue(1.0));
    declare_parameter("min_observed_frames", 3);
    declare_parameter("max_miss_frames", 5);
    declare_parameter("max_age_seconds", 2.0);
    declare_parameter("net_match_distance", 0.05);        
    declare_parameter("net_match_angle", 0.01);         
    declare_parameter("linear_accel_thresh",0.5);
    declare_parameter("angular_accel_thresh", 0.5);
    declare_parameter("pose_window_size", 3);
    declare_parameter("twist_window_size",3);
    declare_parameter("accel_window_size",3);
    declare_parameter("detect_logs", false);     
}

nav2_util::CallbackReturn Localization::on_configure(const rclcpp_lifecycle::State &) {

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    map_frame_id_ = get_parameter("map_frame").as_string();
    odom_frame_id_ = get_parameter("odom_frame").as_string();
    base_link_frame_id_ = get_parameter("base_link_frame").as_string();

    if (map_frame_id_ == odom_frame_id_ || odom_frame_id_ == base_link_frame_id_ || map_frame_id_ == base_link_frame_id_ ) {
        RCLCPP_ERROR(get_logger(),
            "Invalid frame configuration! The values for map_frame, "
            "odom_frame, and base_link_frame must be unique. If using a base_link_frame_output "
            "values, it must not match the map_frame or odom_frame.");
        return nav2_util::CallbackReturn::FAILURE;
    }

    std::string reflector_filename = get_parameter("reflector_filename").as_string();
    // only try to load map if parameter was set
    if (load_reflector_map_from_yaml(reflector_filename)) {
        RCLCPP_INFO(get_logger(),
            "Reflector map loaded successfully from %s, Number  of reflectors: %zu",
            reflector_filename.c_str(), 
            reflectors_.size());
    }

    publish_map_ = get_parameter("publish_map").as_bool();

    double transform_tolerance;
    get_parameter("transform_tolerance", transform_tolerance);
    detect_logs_ = get_parameter("detect_logs").as_bool();

    tolerance_ = get_parameter("tolerance").as_double();
    min_cluster_ = get_parameter("min_cluster").as_int();
    max_cluster_ = get_parameter("max_cluster").as_int();
    min_diameter_ = get_parameter("min_diameter").as_double();
    max_diameter_ = get_parameter("max_diameter").as_double();
    min_intensity_ = get_parameter("min_intensity").as_double();
    max_intensity_ = get_parameter("max_intensity").as_double();
    detect_logs_ = get_parameter("detect_logs").as_bool();
    position_delta_ = get_parameter("position_delta").as_double();
    angle_delta_ = get_parameter("angle_delta").as_double();

    min_confirm_frames_ = get_parameter("min_observed_frames").as_int();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    publish_odom_tf_ = get_parameter("publish_odom_tf").as_bool();
    marking_ = get_parameter("marking").as_bool();
    marking_delta_time_ = get_parameter("marking_delta_time").as_double();
    linear_accel_thresh_ = get_parameter("linear_accel_thresh").as_double();
    angular_accel_thresh_ = get_parameter("angular_accel_thresh").as_double();
    min_observed_frames_ = get_parameter("min_observed_frames").as_int();
    max_miss_frames_ = get_parameter("max_miss_frames").as_int();
    match_distance_thresh_ = get_parameter("match_distance_thresh").as_double();
    max_age_seconds_ = get_parameter("max_age_seconds").as_double();
    net_match_distance_thresh_ = get_parameter("net_match_distance").as_double();
    net_match_angle_thresh_ = get_parameter("net_match_angle").as_double();
    twist_window_size_ = get_parameter("twist_window_size").as_int();
    accel_window_size_ = get_parameter("accel_window_size").as_int();
    pose_window_size_ = get_parameter("pose_window_size").as_int();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface(),
        callback_group_);
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);  // ✨ 关键设置 ✨
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan",
        rclcpp::SensorDataQoS(),[this](const sensor_msgs::msg::LaserScan::SharedPtr scan){
            if (scan->ranges.size() > 3 && active_) {
                scan_callback(scan);
            }
        } ,sub_opt);
    pub_reflector_grid_ = create_publisher<ReflectorGrid>("reflector/recognize", rclcpp::SensorDataQoS());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pub_reflector_network_ = create_publisher<ReflectorGrid>("reflector/network", qos);
    pub_reflector_map_ = create_publisher<ReflectorMap>("reflector/map", qos);
    pub_reflector_matched_ = create_publisher<ReflectorGrid>("reflector/matched", qos);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("reflector_pose",rclcpp::QoS(10));

    load_map_service_ = create_service<LoadReflectorMap>(
        "load_reflector_map",
        std::bind(&Localization::load_reflector_map_callback, this, _1, _2, _3));
        
    get_map_service_ = create_service<GetReflectorMap>(
        "get_reflector_map",
        std::bind(&Localization::get_reflector_map_callback, this, _1, _2, _3));
    
    publish_period_sec_ = get_parameter("publish_period_sec").as_double();
    publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(publish_period_sec_ * 1000)),
        [this]() {
            if (map_available_ && active_) {
                publish_reflector();
            }
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, get_node_base_interface());
    executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
            // 定时清理旧轨迹
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Localization::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating");
    // Keep track of whether we're in the active state. We won't
    // process incoming callbacks until we are
    pub_reflector_matched_->on_activate();
    pub_pose_->on_activate();
    pub_reflector_map_->on_activate();
    pub_reflector_network_->on_activate();
    pub_reflector_grid_->on_activate();

    if (map_available_) {
        publish_reflector();
    }
    // create bond connection
    createBond();
    active_ = true;
  
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Localization::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating");
    active_ = false;
    pub_reflector_matched_->on_deactivate();
    pub_pose_->on_deactivate();
    pub_reflector_map_->on_deactivate();
    pub_reflector_network_->on_deactivate();
    pub_reflector_grid_->on_deactivate();

    // destroy bond connection
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Localization::on_cleanup(const rclcpp_lifecycle::State &)  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    reflector_network_subscription_.reset();
    pub_reflector_matched_.reset();
    pub_pose_.reset();
    tf_broadcaster_.reset();
    pub_reflector_map_.reset();
    pub_reflector_network_.reset();
    load_map_service_.reset();
    reflectors_.clear();
    executor_thread_.reset();
    pub_reflector_grid_.reset();
    map_available_ = false;
    origin_available_ = false;

    tf_listener_.reset();
    tf_buffer_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}


float Localization::compute_cluster_diameter_convex_hull(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) {
    if (cluster->empty()) return 0.0f;
    pcl::ConvexHull<pcl::PointXYZI> hull;
    hull.setInputCloud(cluster);
    pcl::PointCloud<pcl::PointXYZI> hull_points;
    hull.reconstruct(hull_points);
    
    float max_dist = 0.0f;
    for (size_t i = 0; i < hull_points.size(); ++i) {
        for (size_t j = i + 1; j < hull_points.size(); ++j) {
            float dist = pcl::euclideanDistance(hull_points.points[i], hull_points.points[j]);
            max_dist = std::max(max_dist, dist);
        }
    }
    return max_dist;
}

float Localization::compute_intensity_std(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster)  {
    if (cluster->empty()) {
        return 0.0f;
    }

    // 计算均值
    float sum = std::accumulate(
        cluster->begin(), cluster->end(), 0.0f,
        [](float acc, const pcl::PointXYZI& pt) {
            return acc + pt.intensity;
        });
    float mean = sum / cluster->size();

    // 计算方差
    float var = std::accumulate(
        cluster->begin(), cluster->end(), 0.0f,
        [mean](float acc, const pcl::PointXYZI& pt) {
            return acc + std::pow(pt.intensity - mean, 2);
        }) / cluster->size();

    return var == 0 ? mean : std::sqrt(var);
}


std::vector<Reflector> Localization::filter(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &reflector_cloud) {
    // Step 1: 构建KD树用于快速搜索
    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
    kdtree.setInputCloud(reflector_cloud);

    // Step 2: 标记已匹配的点
    std::vector<bool> matched(reflector_cloud->size(), false);

    std::list<PointTrack> tracks = std::move(tracks_);
    // Step 3: 优先匹配现有轨迹
    for (PointTrack& track : tracks) {
        std::vector<int> point_idx(1);
        std::vector<float> point_dist(1);
        if (kdtree.nearestKSearch(track.position, 1, point_idx, point_dist) > 0) {
            float dist = std::sqrt(point_dist[0]);
            if (dist < match_distance_thresh_) {
                // 更新轨迹信息
                track.position = reflector_cloud->points[point_idx[0]];
                track.observed_frames++;
                track.missed_frames = 0;
                track.last_observed = now();
                matched[point_idx[0]] = true;
                continue;
            }
        }
        // 未匹配的轨迹增加丢失计数
        track.missed_frames++;
    }
    
    // Step 4: 创建新轨迹（未匹配的点）
    for (size_t i = 0; i < reflector_cloud->size(); ++i) {
        if (matched[i]) {
            continue;
        }
        // 创建新轨迹
        PointTrack new_track;
        new_track.position = reflector_cloud->points[i];
        new_track.observed_frames = 1;
        new_track.missed_frames = 0;
        new_track.last_observed = now();
        tracks.push_back(new_track);
    }
    // Step 5: 更新轨迹列表
    for(const PointTrack &track:tracks) {
        auto it = std::find_if(tracks_.begin(), tracks_.end(),
            [&](const PointTrack& other) {
                return pcl::euclideanDistance(track.position, other.position) < match_distance_thresh_;
            });
        if (it == tracks_.end()) {
            tracks_.push_back(track);
        }
    }

    std::vector<Reflector> confirmed;
    for (const PointTrack& track : tracks_) {
        if (track.observed_frames >= min_confirm_frames_ && track.missed_frames <= max_miss_frames_) {
            Reflector reflector;
            reflector.point.x = track.position.x;
            reflector.point.y = track.position.y;
            reflector.point.z = 0; // 2D雷达默认z=0
            reflector.intensity = track.position.intensity;
            reflector.diameter = track.position.curvature;
            confirmed.push_back(reflector);
        }
    }
    if (detect_logs_) {
        RCLCPP_INFO(get_logger(), "Confirmed %zu reflectors after filtering.\n", confirmed.size());
    }
    return confirmed;
}  

/*
反射柱点云检测
1. 过滤掉小于最小直径的点
2. 过滤掉强度标准差大于设定值的点
3. 过滤掉大于最大直径的点
4. 过滤掉小于最小聚类数的点
5. 欧式聚类
*/
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Localization::detect_reflectors_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)  {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr reflectors(new pcl::PointCloud<pcl::PointXYZINormal>());
    if (cloud->empty()) {
        return reflectors;
    }
    reflectors->header = cloud->header;
    reflectors->height = cloud->height;
    reflectors->is_dense = cloud->is_dense;
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    /*
    意义：有效簇的最小点数（少于5个点则视为噪声）。
    有效簇的最大点数（防止过大物体被误认为单个反光柱）。典型反光柱在点云中不超过50个点。
    依据：反光柱在10米处约产生5-10个点（SICK TIM571实测）。
    */
    ec.setClusterTolerance(tolerance_); // 两点间距离≤tolerance_则视为同一簇。
    ec.setMinClusterSize(min_cluster_);
    ec.setMaxClusterSize(max_cluster_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    // 几何校验
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, indices, *cluster);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        // 计算直径和强度一致性
        float diameter = compute_cluster_diameter_convex_hull(cluster);
        float intensity_std = compute_intensity_std(cluster);
        if (detect_logs_) {
            RCLCPP_INFO(get_logger(), 
            "Centroid: [x: %f, y: %f, z: %f], Diameter: %f, Intensity StdDev: %f",
            centroid[0], centroid[1], centroid[2], diameter, intensity_std);
        }

        if (diameter > min_diameter_ && diameter < max_diameter_&& intensity_std > min_intensity_ && intensity_std < max_intensity_) {
            pcl::PointXYZINormal point;
            point.x = centroid[0];
            point.y = centroid[1];
            point.z = 0.0;  // 2D雷达默认z=0
            point.intensity = intensity_std;  // 无强度数据时补零
            point.curvature = diameter;  // 无强度数据时补零
            reflectors->push_back(point);
        }
    }

    if (detect_logs_) {
        RCLCPP_INFO(get_logger(), "Detected %zu reflectors.\n", reflectors->size());
    }
    return reflectors;
}
    
pcl::PointCloud<pcl::PointXYZI>::Ptr Localization::laser_scan_to_point_cloud(const sensor_msgs::msg::LaserScan::SharedPtr scan)  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->header.frame_id = scan->header.frame_id;
    cloud->height = 1;
    cloud->is_dense = false;

    // 预分配内存提升性能
    cloud->reserve(scan->ranges.size());

    // 遍历所有激光束
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        // 跳过无效测量值
        if (!std::isfinite(scan->ranges[i])) {
            continue;
        }

        // 计算当前角度（考虑雷达安装偏置）
        float angle = scan->angle_min + i * scan->angle_increment;

        // 转换为笛卡尔坐标
        pcl::PointXYZI point;
        point.x = scan->ranges[i] * std::cos(angle);
        point.y = scan->ranges[i] * std::sin(angle);
        point.z = 0.0;  // 2D雷达默认z=0
        
        // 反射强度处理（不同雷达强度范围不同）
        if (i < scan->intensities.size()) {
            point.intensity = scan->intensities[i];
        } else {
            point.intensity = 0.0;  // 无强度数据时补零
        }

        cloud->push_back(point);
    }

    return cloud;
}

void Localization::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::vector<Reflector> reflectors;
    // 雷达数据转换为点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = laser_scan_to_point_cloud(scan);

    // 反射柱点云检测
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr reflector_cloud = detect_reflectors_cloud(point_cloud);
    if (reflector_cloud->empty()) {
        RCLCPP_WARN(get_logger(), "No reflectors found in the current scan.");
    } else {
        // 反射柱过滤
        // 1. 过滤掉小于最小直径的点
        // 2. 过滤掉强度标准差大于设定值的点
        // 3. 过滤掉大于最大直径的点
        // 4. 过滤掉小于最小聚类数的点
        reflectors = filter(reflector_cloud); // 过滤
        if (reflectors.empty()) {
            RCLCPP_WARN(get_logger(), "No confirmed reflectors after filtering.");
        }
    }
    // 清理旧轨迹
    cleanup_old_tracks();
    reflector_recognize(reflectors,scan->header.frame_id, scan->header.stamp);

    // 反射柱轨迹更新
    ReflectorGrid reflector_grid;
    reflector_grid.header = scan->header;
    reflector_grid.reflectors = reflectors;
    pub_reflector_grid_->publish(reflector_grid);
}

void Localization::cleanup_old_tracks() {
    auto now_time = now();
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
        if (it->missed_frames > max_miss_frames_ || (now_time - it->last_observed).seconds() > max_age_seconds_) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}
void Localization::get_reflector_map_callback(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<GetReflectorMap::Request>,
    std::shared_ptr<GetReflectorMap::Response> response) {
    response->result = GetReflectorMap::Response::RESULT_INVALID_MAP_DATA;
    auto reflectorMap = get_reflector_map();
    if (reflectorMap) {
        response->map = *reflectorMap;
        response->result = GetReflectorMap::Response::RESULT_SUCCESS;
    } 
}

void Localization::load_reflector_map_callback(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<LoadReflectorMap::Request> request,
    std::shared_ptr<LoadReflectorMap::Response> response) {
    response->result = LoadReflectorMap::Response::RESULT_INVALID_MAP_DATA;
    if (load_reflector_map(request->map)) {
        publish_reflector();
        response->result = LoadReflectorMap::Response::RESULT_SUCCESS;
    }
}


std::unique_ptr<orient_interfaces::msg::ReflectorMap> Localization::get_reflector_map() {
    auto reflectorMap = std::make_unique<orient_interfaces::msg::ReflectorMap>();
    auto [min_x, max_x, min_y, max_y] = get_reflector_round_bounds(reflectors_);
    reflectorMap->origin = origin_;
    reflectorMap->reflectors = reflectors_;
    for(Reflector &reflector : reflectorMap->reflectors) {
        reflector.point.x -= origin_.position.x;
        reflector.point.y -= origin_.position.y;
    }
    return reflectorMap;
}

bool Localization::load_reflector_map(const orient_interfaces::msg::ReflectorMap &reflectorMap) {

    auto origin = reflectorMap.origin.position;
    origin_.position.x = origin.x;
    origin_.position.y = origin.y;
    origin_.position.z = 0.0;
    origin_.orientation = orientationAroundZAxis(origin.z);
    origin_available_ = true;
    reflectors_ = reflectorMap.reflectors;
    for(Reflector &reflector : reflectors_) {
        reflector.point.x += origin_.position.x;
        reflector.point.y += origin_.position.y;
    }
    map_available_ = true;
    rebuild_global_nets();
    return true;
}

bool Localization::load_reflector_map_from_yaml(const std::string &yaml_file ) {
    orient_interfaces::msg::ReflectorMap reflector_map;
    std::ifstream fin(yaml_file);
    if (!fin) {
        return false;
    }
    fin >> reflector_map;
    return load_reflector_map(reflector_map);
}

void Localization::publish_tf(const geometry_msgs::msg::Pose &robot_pose, const rclcpp::Time &stamp){

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_id_;
    map_to_odom.child_frame_id = odom_frame_id_;
    map_to_odom.transform = to_geometry_msg_transform(robot_pose);
    transforms.push_back(map_to_odom);

    if (publish_odom_tf_) {
        map_to_odom.header.frame_id = odom_frame_id_;
        map_to_odom.child_frame_id = base_link_frame_id_;
        map_to_odom.transform = to_geometry_msg_transform(calculate_odometry_pose(robot_pose,robot_pose));
        transforms.push_back(map_to_odom);
    }
    tf_broadcaster_->sendTransform(transforms);
}

void Localization::publish_reflector() {
    publish_network();
    if (publish_map_ && map_available_) {
        publish_map();
    }
}

void Localization::publish_map() {
    auto reflectorMap = get_reflector_map();
    if ( pub_reflector_map_->get_subscription_count() > 0 && reflectorMap) {
        pub_reflector_map_->publish(std::move(reflectorMap));
    }

}

void Localization::publish_network() {
    if (pub_reflector_network_->get_subscription_count() == 0) {
        return;
    }
    ReflectorGrid reflectorGrid;
    reflectorGrid.header.frame_id = map_frame_id_;
    reflectorGrid.header.stamp = now();
    reflectorGrid.reflectors = reflectors_;
    pub_reflector_network_->publish(reflectorGrid); 
}

nav2_util::CallbackReturn Localization::on_shutdown(const rclcpp_lifecycle::State &)  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

//比较器 最优匹配结果排序
bool Localization::compare_match_result(MatchResult &m1, MatchResult &m2) {
    if (m1.matched.size() > m2.matched.size()) {
        return true;
    }
    if (m1.matched.size() == m2.matched.size() && m1.cost < m2.cost && m1.cost >= 0) {
        return true;
    }
    return false;
}


    /**
 * 找出最靠近point的全局反光柱near
 * @param point
 * @param near
 * @return 距离
 */
double Localization::nearest(const geometry_msgs::msg::Point &point, Reflector &near) const {
    double dist = 999999;
    for (const Reflector &reflector: reflectors_) {
        double temp = nav2_util::geometry_utils::euclidean_distance(point, reflector.point);
        if (temp < dist) {
            dist = temp;
            near = reflector;
        }
    }
    return dist;
}

/**
 * 获取局部反光柱经过旋转平移变换后的匹配得分
 * @param reflectors_local
 * @param rotate 旋转
 * @param tran 平移
 * @return
 */
MatchResult Localization::get_matching_score(const std::vector<Reflector> &reflectors_local, double yaw, const geometry_msgs::msg::Point &tran) {
    MatchResult result;
    for (size_t i = 0; i < reflectors_local.size(); ++i) {
        geometry_msgs::msg::Point point_in_global = rotate(reflectors_local[i].point, yaw) + tran;
        Reflector near_reflector;
        double dist = nearest(point_in_global, near_reflector);
        if (dist < match_distance_thresh_) {
            result.cost += dist;
            result.matched.push_back({reflectors_local[i], near_reflector});
        }
    }
    if (result.matched.empty()) {
        result.cost = -1;
    } else {
        result.cost /= static_cast<double >(result.matched.size());
    }

    return result;
}


geometry_msgs::msg::Point Localization::rotate(const geometry_msgs::msg::Point &point, double yaw) {
    geometry_msgs::msg::Point result;
    result.x = point.x * std::cos(yaw) - point.y * std::sin(yaw);
    result.y = point.x * std::sin(yaw) + point.y * std::cos(yaw);
    return result;
}

double Localization::get_theta(const geometry_msgs::msg::Point &p) {
    return std::atan2(p.y, p.x);
}
/**
 * 这段代码的主要功能是：
    1. 计算局部和全局反射器之间的旋转角度和平移向量。
    2. 根据这些参数计算匹配分数。
    3. 筛选出符合条件的匹配结果，并存储到 `results` 容器中。
    * @param reflectors_local 局部反光柱集合
    * @param result
    * @return
    */
bool Localization::reflector_matching(const std::vector<Reflector> &reflectors_local, MatchResult &matched) {
    std::vector<MatchResult> results;

    /*
    用于处理反射器（`Reflector`）之间的匹配逻辑。它的核心功能是计算两个反射器集合之间的匹配分数，并将符合条件的匹配结果存储到 `results` 容器中。
    具体来说，它会遍历局部反射器集合和全局反射器集合，计算它们之间的匹配分数，并将符合条件的匹配结果存储到 `results` 容器中。
    该函数的输入参数包括局部反射器集合 `reflectors_local` 和全局反射器集合 `reflectors_`，以及一个用于存储匹配结果的 `matched` 对象。
    匹配结果包括匹配的反射器对和匹配的分数。
    匹配的分数是通过计算局部反射器和全局反射器之间的距离来获得的。匹配的反射器对是通过遍历局部反射器集合和全局反射器集合，计算它们之间的匹配分数来获得的。
    匹配的分数越小，匹配的结果越好。
    匹配的结果是通过 `get_matching_score` 函数来计算的，该函数会遍历局部反射器集合，计算它们与全局反射器之间的距离，并返回匹配的分数和匹配的反射器对。

    */
    auto matcher = [&](const Reflector &A_local, const Reflector &A_global, const Reflector &B_local, const Reflector &B_global) -> void {
        /*
            计算全局坐标系下两个反射器之间的向量。yaw表示全局坐标系和局部坐标系之间的旋转角度差。
        */
        double yaw = get_theta(B_global.point - A_global.point) - get_theta(B_local.point - A_local.point);

        //将局部坐标系中的点 `A_local.point` 按照 `yaw` 角度旋转。
        //计算全局坐标系下的平移向量 `tran`，用于将局部坐标系对齐到全局坐标系。
        geometry_msgs::msg::Point tran = A_global.point - rotate(A_local.point, yaw);

        //用于根据旋转角度 `yaw` 和平移向量 `tran` 计算匹配分数。
        MatchResult matchResult = get_matching_score(reflectors_local, yaw, tran);

        //匹配代价必须是非负值。匹配的反射器对数量必须大于 2。
        if (matchResult.cost >= 0 && matchResult.matched.size() > 2) {
            results.emplace_back(matchResult);
        }
    };
    std::vector<Net> nets_local = make_reflector_net(reflectors_local);     //当前各个反光柱之间的距离
    for (const Net &net_local: nets_local) {
        for (const Net &net_global: nets_global_) {
            if (net_local.is_similar(net_global, net_match_distance_thresh_, net_match_angle_thresh_)) {
                // 计算局部反光柱和全局反光柱之间的匹配
                // 1. A_local -> A_global
                // 2. B_local -> B_global
                matcher(reflectors_local[net_local.first], reflectors_[net_global.first], reflectors_local[net_local.second], reflectors_[net_global.second]);
                matcher(reflectors_local[net_local.first], reflectors_[net_global.second], reflectors_local[net_local.second], reflectors_[net_global.first]);
            }
        }
    }
    if (results.size() > 0) {
        std::sort(results.begin(), results.end(), compare_match_result);
        matched = std::move(results.front());
    }
    return results.size() > 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Localization::convert_point_cloud(const std::vector<Reflector> &in_reflectors) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->height = 1;
    cloud->is_dense = false;
    // 预分配内存提升性能
    cloud->reserve(in_reflectors.size());
    for (size_t i = 0; i < in_reflectors.size(); ++i) {
        cloud->push_back(convert_point(in_reflectors[i]));
    }
    return cloud;
}
pcl::PointXYZ Localization::convert_point(const Reflector &reflector) {
    pcl::PointXYZ point;
    point.x = reflector.point.x;
    point.y = reflector.point.y;
    point.z = 0.0;  // 2D雷达默认z=0
    return point;
}
geometry_msgs::msg::Point Localization::registration(
    const geometry_msgs::msg::Pose &robot_pose, const geometry_msgs::msg::Point &in_point) const {
    geometry_msgs::msg::Point point;
    double rx = robot_pose.position.x;
    double ry = robot_pose.position.y;
    double yaw = tf2::getYaw(robot_pose.orientation);
    double cos_theta = cos(yaw);
    double sin_theta = sin(yaw);
    point.z =in_point.z;
    point.x = rx + in_point.x * cos_theta - in_point.y* sin_theta;
    point.y = ry + in_point.x * sin_theta + in_point.y * cos_theta;
    return point;
}

void Localization::verify_reflector_tracks(const geometry_msgs::msg::Pose &robot_pose, const std::vector<Reflector> &in_reflectors) {
    // Step 1: 过滤掉已经存在的反光柱
    std::vector<Reflector> reflectors;
    reflectors.reserve(in_reflectors.size());

    for (const Reflector &in_reflector : in_reflectors) {
        Reflector new_reflector = in_reflector;

        // 将反光柱坐标对齐到全局坐标系
        // 这里的 registration 函数是将局部坐标系下的反光柱坐标转换到全局坐标系
        new_reflector.point = registration(robot_pose, in_reflector.point);

        int idx = 0;
        float dist = 0;
        if (nearest_global_reflector(new_reflector, idx, dist) && dist < match_distance_thresh_) {
            // 如果反光柱已经存在于全局列表中，则不添加
            continue;
        }
        // 如果反光柱不在全局列表中，则添加到新的反光柱列表中
        reflectors.push_back(new_reflector);
    }
    if (reflectors.empty()) {
        // 如果没有新的反光柱，则直接返回
        return;
    }

    // Step 2: 转换为点云
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(convert_point_cloud(reflectors));
    
    // Step 3: 标记已匹配的点
    std::list<TrackedReflector> reflector_tracks = std::move(reflector_tracks_);
    std::vector<bool> matched(reflectors.size(), false);

    // Step 3: 优先匹配现有轨迹
    for (TrackedReflector &track : reflector_tracks) {
        std::vector<int> point_idx(1);
        std::vector<float> point_dist(1);
        if (kdtree.nearestKSearch(convert_point(track.reflector), 1, point_idx, point_dist) > 0) {
            float dist = std::sqrt(point_dist[0]);
            if (dist < match_distance_thresh_) {
                // 更新轨迹信息
                track.reflector = reflectors[point_idx[0]];
                track.missed_frames = 0;
                track.observed_frames++;
                track.last_observed = now();
                matched[point_idx[0]] = true;
                continue;
            }
        }
        // 未匹配的轨迹增加丢失计数
        track.missed_frames++;
    }

    // Step 4: 创建新轨迹（未匹配的点）
    for (size_t i = 0; i < reflectors.size(); ++i) {
        if (matched[i]) {
            continue;
        }
        TrackedReflector new_track;
        new_track.reflector = reflectors[i];
        new_track.observed_frames = 1;
        new_track.missed_frames = 0;
        new_track.last_observed = now();
        reflector_tracks.push_back(new_track);
    }

    // Step 5: 更新轨迹列表
    for(const auto &track:reflector_tracks) {
        auto it = std::find_if(reflector_tracks_.begin(), reflector_tracks_.end(),
            [&](const TrackedReflector& other) {
                return orient_reflector::euclidean_distance(track.reflector, other.reflector) < match_distance_thresh_;
            });
        if (it == reflector_tracks_.end()) {
            reflector_tracks_.push_back(track);
        }
    }
}

void Localization::cleanup_old_reflector() {
    auto now_time = now();
    for (auto it = reflector_tracks_.begin(); it != reflector_tracks_.end(); ) {
        if (it->missed_frames > max_miss_frames_ ||
            it->observed_frames > min_observed_frames_ ||  
            (now_time - it->last_observed).seconds() > max_age_seconds_) {
            it = reflector_tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

bool Localization::confirm_reflectors() { 
    bool changed = false;
    for (const TrackedReflector &track : reflector_tracks_) {
        if (track.observed_frames < min_observed_frames_ || track.missed_frames > max_miss_frames_)  {
            continue;
        }
        if (!global_reflector_existed(track.reflector)) {
            changed = true;
            reflectors_.push_back(track.reflector);
            RCLCPP_INFO(this->get_logger(), 
                "New reflector added to the global map : (x: %.3f, y: %.3f, diameter: %.4f, intensity: %.5f)", 
                track.reflector.point.x, 
                track.reflector.point.y,
                track.reflector.diameter,
                track.reflector.intensity);
        }
    }
    return changed;
}

bool Localization::is_accelerated(const geometry_msgs::msg::Twist &, const geometry_msgs::msg::Accel &accel) {
    if (std::abs(accel.linear.x) > linear_accel_thresh_ || std::abs(accel.angular.z) > angular_accel_thresh_) {
        return true;
    }
    return false;
}


void Localization::stamp_reflectors(const geometry_msgs::msg::Pose &robot_pose,const std::vector<Reflector> &in_reflectors) {

    // Step 1: 计算机器人速度
    // 机器人速度过快则不进行反光柱匹配
    if (is_accelerated(twist_.twist,accel_.accel)) {

        return;
    }
    // Step 2: 过滤掉小于最小直径的点
    verify_reflector_tracks(robot_pose, in_reflectors);

    // Step 3: 多帧确认检测反光柱
    bool changed = confirm_reflectors();

    cleanup_old_tracks();
    
    if (changed && !origin_available_) {
        if (reflectors_.size() > 3) {
            auto [min_x, max_x, min_y, max_y] = get_reflector_round_bounds(reflectors_);
            origin_ = trilateration_from_reflector(reflectors_);
            origin_.position.x = min_x ;
            origin_.position.y = min_y ;
            origin_available_ = true;
            map_available_ = reflectors_.size() > 0;
        }
    }
    if (changed && active_) {
        rebuild_global_nets();
        publish_reflector();
        RCLCPP_INFO(this->get_logger(), "Reflector map updated. Total reflectors: %zu", reflectors_.size());
    }

    
}

class PoseResidual {
public:
    PoseResidual(const geometry_msgs::msg::Point &global,const geometry_msgs::msg::Point &local)
        : global_(global), local_(local){
    }

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose[0]=x, pose[1]=y, pose[2]=theta
        T x_proj = pose[0] + T(local_.x) * cos(pose[2]) - T(local_.y) * sin(pose[2]);
        T y_proj = pose[1] + T(local_.x) * sin(pose[2]) + T(local_.y) * cos(pose[2]);
        residual[0] = x_proj - T(global_.x);
        residual[1] = y_proj - T(global_.y);
        return true;
    }

private:
    geometry_msgs::msg::Point global_;
    geometry_msgs::msg::Point local_;
};
    
geometry_msgs::msg::Pose Localization::locate_robot_pose(const std::vector<Matched> &matcheds) {
    // 1. 计算初始位姿
    geometry_msgs::msg::Pose robot_pose = trilateration_from_matched(matcheds);
    
    // 2. 使用Ceres求解器优化位姿
    // double pose[3]{
    //     robot_pose.position.x, 
    //     robot_pose.position.y, 
    //     tf2::getYaw(robot_pose.orientation)};
    // ceres::Problem problem;
    // for(const Matched &matched: matcheds) {
    //     ceres::CostFunction* cost_function = 
    //         new ceres::AutoDiffCostFunction<PoseResidual, 2, 3>(
    //             new PoseResidual(matched.global.point, matched.local.point));
    //     problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.5) , pose);
    // }
    // // 配置求解器
    // ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_QR;
    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);
    // robot_pose.position.x = pose[0] ;
    // robot_pose.position.y = pose[1] ;
    // robot_pose.orientation = orientationAroundZAxis(pose[2]);
    return robot_pose;
}

geometry_msgs::msg::Pose Localization::trilateration_from_matched(const std::vector<Matched> &matched) {
    // 创建两个动态大小的矩阵，分别存储全局和局部坐标
    Eigen::Matrix<double, 2, Eigen::Dynamic> globals(2, matched.size());
    Eigen::Matrix<double, 2, Eigen::Dynamic> locals(2, matched.size());

    // 将匹配的点的全局和局部坐标填入矩阵
    for (size_t i = 0; i < matched.size(); i++) {
        globals(0, i) = matched[i].global.point.x;
        globals(1, i) = matched[i].global.point.y;
        locals(0, i) = matched[i].local.point.x;
        locals(1, i) = matched[i].local.point.y;
    }

    // 使用 umeyama 方法计算从局部坐标系到全局坐标系的变换矩阵
    Eigen::Matrix3d m = Eigen::umeyama(locals, globals, false);
    geometry_msgs::msg::Pose pose;
    pose.position.x = m(0, 2);
    pose.position.y = m(1, 2);
    pose.position.z = 0;
    double half_yaw = std::atan2(m(1, 0), m(0, 0)) * 0.5;  
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(half_yaw); 
    pose.orientation.w = std::cos(half_yaw); 
    return pose;
}

void Localization::reflector_recognize(const std::vector<Reflector> &reflectors, const std::string &frame_id, const rclcpp::Time &recognize_stamp) {
    bool located = false;

    geometry_msgs::msg::Pose robot_pose = robot_pose_.pose;
    if (reflectors_.empty()) {
        located = true;
        robot_pose.orientation = orientationAroundZAxis(robot_pose.orientation.z);
        RCLCPP_INFO(this->get_logger(), "No global reflectors available for matching. set initial pose.");
    } else if (reflectors.size() >= 3) {
        MatchResult matched;
        if (reflector_matching(reflectors, matched)) {
            robot_pose = locate_robot_pose(matched.matched);
            publish_matched(matched.matched, recognize_stamp);
            located = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "No matching reflectors found.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Not enough reflectors for matching. %ld", reflectors.size());
    }
    robot_pose_.header.frame_id = frame_id;

    // 计算机器人在base_link坐标系下的速度
    compute_velocity(robot_pose, recognize_stamp);
    robot_pose_.pose = robot_pose;
    robot_pose_.header.stamp = recognize_stamp;

    if (located && marking_) {
        stamp_reflectors(robot_pose_.pose, reflectors);
    }
    try {
        // 计算机器人在base_link坐标系下的位姿
        geometry_msgs::msg::Pose robot_base_link_pose = convert_to_base_link(robot_pose_.pose, frame_id);
        if (publish_tf_) {
            publish_tf(robot_base_link_pose,recognize_stamp);
        }
        if (located){
            publish_pose(robot_base_link_pose, recognize_stamp);
        }
    } catch(tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not get transform  %s", ex.what());
    }
}

geometry_msgs::msg::Pose Localization::convert_to_base_link(const geometry_msgs::msg::Pose& robot_pose, const std::string &frame_id) {
    tf2::Transform tf_map_to_scanner;
    tf2::fromMsg(robot_pose, tf_map_to_scanner);
    tf2::Transform tf_map_to_base_link;
    auto base_link_to_scanner = tf_buffer_->lookupTransform(frame_id, base_link_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(base_link_to_scanner.transform, tf_map_to_base_link);
    tf2::Transform base_translation;
    base_translation.mult(tf_map_to_scanner, tf_map_to_base_link);
    geometry_msgs::msg::Pose robot_base_link_pose;
    tf2::toMsg(base_translation, robot_base_link_pose);
    return robot_base_link_pose;
}


void Localization::publish_matched(const std::vector<Matched> &matcheds, const rclcpp::Time &stamp) {
    if (pub_reflector_matched_->get_subscription_count() <= 0) {
        return;
    }
    ReflectorGrid reflector_grid;
    reflector_grid.header.frame_id = map_frame_id_;
    reflector_grid.header.stamp = stamp;
        for (const Matched &matched: matcheds) {
        reflector_grid.reflectors.push_back(matched.global);
    }
    pub_reflector_matched_->publish(reflector_grid);
}

geometry_msgs::msg::Transform Localization::to_geometry_msg_transform(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::Transform transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;
    return transform;
}

geometry_msgs::msg::Pose Localization::calculate_odometry_pose( const geometry_msgs::msg::Pose& initial_pose,  const geometry_msgs::msg::Pose& current_pose)  const{
    // Create transforms from poses  
    tf2::Transform transform1, transform2;  
    transform1.setOrigin({initial_pose.position.x, initial_pose.position.y, initial_pose.position.z});  
    transform1.setRotation(tf2::Quaternion(initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w));  

    transform2.setOrigin(tf2::Vector3(current_pose.position.x, current_pose.position.y, current_pose.position.z));  
    transform2.setRotation(tf2::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));  

    // Calculate the relative transform  
    tf2::Transform relative_transform = transform1.inverse() * transform2;  

    // Convert the relative transform back to a Pose  
    geometry_msgs::msg::Pose relative_pose;  
    relative_pose.position.x = relative_transform.getOrigin().x();  
    relative_pose.position.y = relative_transform.getOrigin().y();  
    relative_pose.position.z = relative_transform.getOrigin().z();  

    tf2::Quaternion relative_orientation = relative_transform.getRotation();  
    relative_pose.orientation.x = relative_orientation.x();  
    relative_pose.orientation.y = relative_orientation.y();  
    relative_pose.orientation.z = relative_orientation.z();  
    relative_pose.orientation.w = relative_orientation.w();

    return relative_pose;  
}  

geometry_msgs::msg::PoseWithCovarianceStamped Localization::convert_pose_to_pose(const geometry_msgs::msg::Pose &robot_pose,const rclcpp::Time &stamp) const {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.pose.pose = robot_pose;
    return pose;
}


void Localization::publish_pose(const geometry_msgs::msg::Pose &robot,const rclcpp::Time &stamp)  const{
    geometry_msgs::msg::PoseWithCovarianceStamped pose = convert_pose_to_pose(robot, stamp); 
     // 协方差矩阵 (体现毫米级精度)
    std::array<double, 36> cov = {};
    cov[0] = 0.0001;  // x方差 (0.01cm²)
    cov[7] = 0.0001;  // y方差
    cov[35] = 0.0001; // yaw方差 (0.01rad²)
    pose.pose.covariance = cov;
    pose.header.frame_id = map_frame_id_; 
    pub_pose_->publish(pose);  
}


void Localization::rebuild_global_nets() {
    if (reflectors_.empty()) {
        return;
    }

    // 重新构建全局反光柱网络
    kdtree_reflectors_.setInputCloud(convert_point_cloud(reflectors_));
    //全局节点网络
    nets_global_ = make_reflector_net(reflectors_);
}

bool Localization::global_reflector_existed(const Reflector &reflector) {
    int idx = 0;
    float dist = 0;
    return nearest_global_reflector(reflector, idx, dist) && dist < match_distance_thresh_;
}

bool Localization::nearest_global_reflector(const Reflector &reflector, int &near, float &dist) {
    std::vector<int> point_idx(1);
    std::vector<float> point_dist(1);
    if (kdtree_reflectors_.nearestKSearch(convert_point(reflector), 1, point_idx, point_dist) > 0) {
        near = point_idx[0];
        dist = std::sqrt(point_dist[0]);
        return true;
    }
    return false;
}

std::vector<Net>  Localization::make_reflector_net(const std::vector<Reflector> &reflectors) {
    std::vector<Net> nets_local;     //当前各个反光柱之间的距离
    //局部节点网络 实时构建
    for (size_t i = 1; i < reflectors.size(); i++) {//遍历当前各个反光柱圆心的坐标距离
        for (size_t j = 0; j < i; j++) {
            float distance = orient_reflector::euclidean_distance(reflectors[i], reflectors[j]);
            float angle = atan2f(reflectors[i].point.y - reflectors[j].point.y, reflectors[i].point.x - reflectors[j].point.x);
            nets_local.emplace_back(Net{i, j, distance, angle});
        }
    }
    return nets_local;
}


geometry_msgs::msg::Pose Localization::calculate_pose(const geometry_msgs::msg::Pose &p1) {
    pose_window_.push_back(p1);
    if (pose_window_.size() > pose_window_size_) {
        pose_window_.erase(pose_window_.begin());
    }
    geometry_msgs::msg::Pose sum_pose;
    for(const geometry_msgs::msg::Pose &pose: pose_window_) {
        sum_pose.position.x += pose.position.x;
        sum_pose.position.y += pose.position.y;
        sum_pose.position.z += pose.position.z;
        sum_pose.orientation.x += pose.orientation.x;
        sum_pose.orientation.y += pose.orientation.y;
        sum_pose.orientation.z += pose.orientation.z;
        sum_pose.orientation.w += pose.orientation.w;
    }
    sum_pose.position.x = sum_pose.position.x / pose_window_.size();
    sum_pose.position.y = sum_pose.position.y / pose_window_.size();
    sum_pose.position.z = sum_pose.position.z / pose_window_.size();
    sum_pose.orientation.x = sum_pose.orientation.x / pose_window_.size();
    sum_pose.orientation.y = sum_pose.orientation.y / pose_window_.size();
    sum_pose.orientation.z = sum_pose.orientation.z / pose_window_.size();
    sum_pose.orientation.w = sum_pose.orientation.w / pose_window_.size();
    return sum_pose;
}

geometry_msgs::msg::Twist Localization::calculate_twist(const geometry_msgs::msg::Pose &p1,const geometry_msgs::msg::Pose &p2, double delta_time) {
    geometry_msgs::msg::Twist twist;
    double delta_x = p1.position.x - p2.position.x;
    double delta_y = p1.position.y - p2.position.y;
    if (delta_time > 0) {
        double linear_velocity = std::sqrt(delta_x * delta_x + delta_y * delta_y) / delta_time;
        double delta_yaw = tf2::getYaw(p1.orientation) - tf2::getYaw(p2.orientation);
        double angular_velocity = delta_yaw / delta_time;
        twist.linear.x = std::abs(linear_velocity);
        twist.angular.z = std::abs(angular_velocity);
    } else {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }
    return twist;   
}


geometry_msgs::msg::Twist Localization::calculate_twist(const geometry_msgs::msg::Pose &robot_pose, double delta_time) {
    geometry_msgs::msg::Twist twist = calculate_twist(robot_pose_.pose, robot_pose, delta_time);
    twists_window_.push_back(twist);
    if (twists_window_.size() > twist_window_size_) {
        twists_window_.erase(twists_window_.begin());
    }
    geometry_msgs::msg::Twist sum_twist;
    for(const geometry_msgs::msg::Twist &twist: twists_window_) {
        sum_twist.linear.x += twist.linear.x;
        sum_twist.angular.z += twist.angular.z;
    }
    twist.linear.x = sum_twist.linear.x / twists_window_.size();
    twist.angular.z = sum_twist.angular.z / twists_window_.size();
    return twist;   
}


geometry_msgs::msg::Accel Localization::calculate_accel(const geometry_msgs::msg::Twist &t1,const geometry_msgs::msg::Twist &t2, double delta_time) {
    geometry_msgs::msg::Accel accel;
    if (delta_time > 0) {
        accel.linear.x = (t1.linear.x - t2.linear.x) / delta_time;
        accel.angular.z = (t1.angular.z - t2.angular.z) / delta_time;
    } else {
        accel.linear.x = 0.0;
        accel.angular.z = 0.0;
    }
    return accel;   
}

geometry_msgs::msg::Accel Localization::calculate_accel(const geometry_msgs::msg::Twist &t1, double delta_time) {
    geometry_msgs::msg::Accel accel = calculate_accel(twist_.twist, t1, delta_time);
    accels_window_.push_back(accel);
    if (accels_window_.size() > accel_window_size_) {
        accels_window_.erase(accels_window_.begin());
    }
    geometry_msgs::msg::Accel sum_accel;
    for(const geometry_msgs::msg::Accel &accel: accels_window_) {
        sum_accel.linear.x += accel.linear.x;
        sum_accel.angular.z += accel.angular.z;
    }
    accel.linear.x = sum_accel.linear.x / accels_window_.size();
    accel.angular.z = sum_accel.angular.z / accels_window_.size();
    return accel;   
}

void Localization::compute_velocity(const geometry_msgs::msg::Pose &robot_pose, const rclcpp::Time &stamp) {
    accel_.header.stamp = twist_.header.stamp = stamp;
    accel_.header.frame_id = twist_.header.frame_id = robot_pose_.header.frame_id;
    double delta_time = (stamp - robot_pose_.header.stamp).seconds();
    geometry_msgs::msg::Twist current_twist =  calculate_twist(robot_pose, delta_time);
    accel_.accel = calculate_accel(current_twist, delta_time);
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(orient_reflector::Localization)
