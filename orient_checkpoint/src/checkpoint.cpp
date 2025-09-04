

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <fstream>
#include <optional>
#include <cstdlib>
#include <libgen.h>
#include <Magick++.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rcpputils/env.hpp>
#include "nav2_navfn_planner/navfn.hpp"
#include "yaml-cpp/yaml.h"

#include "nav2_msgs/msg/costmap.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_smoother/smoother_utils.hpp"
#include "orient_checkpoint/checkpoint.hpp"
#include <unordered_set>
#include "orient_utils/transform.hpp"
#include "orient_utils/estimate.hpp"

using namespace std::placeholders;
using namespace orient_interfaces::msg;

namespace orient_checkpoint {

using namespace smoother_utils;  // NOLINT
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT

CheckpointServer::CheckpointServer(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("checkpoint_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");
  declare_parameter("orient_clientid", rcpputils::get_env_var("ORIENT_CLIENTID"));
  declare_parameter("global_frame", "map");
  declare_parameter("robot_base_frame","base_footprint");
  declare_parameter("nav_graph_file", "");
  declare_parameter("map", "");
  declare_parameter("frequency", 0.5);
  declare_parameter("match_distance_thresh", 0.7);        
  declare_parameter("transform_tolerance", 0.1);
  declare_parameter("tolerance", rclcpp::ParameterValue(1e-10));
  declare_parameter("max_its", rclcpp::ParameterValue(1000));
  declare_parameter("w_data", rclcpp::ParameterValue(0.9));
  declare_parameter("w_smooth", rclcpp::ParameterValue(0.5));
  declare_parameter("do_refinement", rclcpp::ParameterValue(true));
  declare_parameter("allow_reversing", rclcpp::ParameterValue(false));
  declare_parameter("refinement_num", rclcpp::ParameterValue(2));
  declare_parameter("interpolation_resolution", 0.5);    
  current_check_point_.index = std::numeric_limits<uint32_t>::max();
}

nav2_util::CallbackReturn CheckpointServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  orient_clientid_ = get_parameter("orient_clientid").as_string();
  if (orient_clientid_.empty()) {
      throw std::runtime_error("ORIENT_CLIENTID environment variable not set");
  }
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  std::string map_file = get_parameter("map").as_string();
  if (!map_file.empty()) {
      if (!loadMapFromYaml(map_file)) {
        throw std::runtime_error("Failed to load map from file: " + map_file);
      }
  }

  std::string nav_graph_file = get_parameter("nav_graph_file").as_string();
  if (!nav_graph_file.empty()) {
      if (loadCheckPointFromYaml(nav_graph_file)) {
          RCLCPP_INFO(get_logger(), 
            "Loaded checkpoint map from file %s with %zu points and %zu paths", 
            nav_graph_file.c_str(),checkpoint_map_.points.size(), checkpoint_map_.paths.size());
      }
  }
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  match_distance_thresh_ = get_parameter("match_distance_thresh").as_double();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  interpolation_resolution_ = get_parameter("interpolation_resolution").as_double();

  get_parameter("tolerance", tolerance_);
  get_parameter("max_its", max_its_);
  get_parameter("w_data", data_w_);
  get_parameter("w_smooth", smooth_w_);
  get_parameter("do_refinement", do_refinement_);
  get_parameter("refinement_num", refinement_num_);
  allow_reversing_ = get_parameter("allow_reversing").as_bool();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface(),
      callback_group_);
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  appraise_checkpoint_ = create_service<orient_interfaces::srv::CheckPointPose>(
    "checkpoint_appraise",
    std::bind(&CheckpointServer::appraise_checkpoint, this, _1, _2, _3));

  get_map_service_ = create_service<orient_interfaces::srv::GetCheckPointMap>(
      "get_checkpoint_map",
      std::bind(&CheckpointServer::get_checkpoint_map_callback, this, _1, _2, _3));
  
  // Create a service that loads the occupancy grid from a file
  load_checkpoint_service_ = create_service<orient_interfaces::srv::LoadCheckPoint>(
      "load_checkpoint",
      std::bind(&CheckpointServer::loadCheckPointCallback, this, _1, _2, _3));

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  checkpoint_state_update_sub_ = create_subscription<orient_interfaces::msg::CheckPointStateMap>(
      "checkpoint/state/update", rclcpp::ServicesQoS(),
      std::bind(&CheckpointServer::checkpoint_state_update, this, std::placeholders::_1),sub_opt);

  checkpoint_publisher_ = create_publisher<orient_interfaces::msg::CheckPointStateMap>("checkpoints", 1);
  current_checkpoint_publisher_ = create_publisher<orient_interfaces::msg::CheckPointStateStamp>("checkpoint/current", rclcpp::ServicesQoS());
  current_position_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("checkpoint_pose", rclcpp::QoS(10));

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);
  global_plan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud>("checkpoints/path", 1);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(),
    "compute_path_through_poses",
    std::bind(&CheckpointServer::computePlanThroughPoses, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  frequency_ = get_parameter("frequency").as_double();
  current_checkpoint_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(frequency_ * 1000)),
      [this]() {
        if (active_) {
          check_current_checkpoint();
        }
      },callback_group_);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn CheckpointServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  active_ = true;
  action_server_poses_->activate();
  plan_publisher_->on_activate();
  global_plan_publisher_->on_activate();
  current_checkpoint_publisher_->on_activate();
  checkpoint_publisher_->on_activate();
  current_position_publisher_->on_activate();
  if (checkpoint_map_.paths.size() > 0) {
    publish_checkpoint_map();
    publish_checkpoint_path();
  }
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CheckpointServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  active_ = false;
  plan_publisher_->on_deactivate();
  global_plan_publisher_->on_deactivate();
  action_server_poses_->deactivate();
  current_checkpoint_publisher_->on_deactivate();
  checkpoint_publisher_->on_deactivate();
  current_position_publisher_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CheckpointServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  global_checkpoints_map_.clear();
  vertex_map_.clear();
  graph_.clear();
  checkpoint_map_.points.clear();
  checkpoint_map_.paths.clear();
  appraise_checkpoint_.reset();
  load_checkpoint_service_.reset();
  current_checkpoint_publisher_.reset();
  checkpoint_publisher_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  global_plan_publisher_.reset();
  executor_thread_.reset();
  current_position_publisher_.reset();
  tf_.reset();
  planner_.reset();
  costmap_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CheckpointServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void CheckpointServer::check_current_checkpoint() {
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) {
      return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = current_pose.header;
  pose_msg.pose.pose = current_pose.pose;
  current_position_publisher_->publish(pose_msg);

  orient_interfaces::msg::CheckPointState current_check_point_state;
  orient_interfaces::msg::CheckPoint current_check_point;
  if (get_checkpoint(current_pose.pose.position, current_check_point)) {
    current_check_point_state = global_checkpoints_map_[current_check_point];
  } else {
    current_check_point.index = std::numeric_limits<uint32_t>::max();
    current_check_point_state.point = current_check_point;
  }
  current_check_point_ = current_check_point;

  publish_current_checkpoint(current_check_point_state);
}

void CheckpointServer::checkpoint_state_update(const CheckPointStateMap::SharedPtr checkpoint_state) {
  for(const auto &checkpoint : checkpoint_state->checkpoints) {
    auto iter = global_checkpoints_map_.find(checkpoint.point);
    if (iter == std::end(global_checkpoints_map_)) {
        continue;
    }
    if (checkpoint.state) {
      if (iter->second.clientid.empty()) {
        iter->second.clientid = checkpoint.clientid ;
      }
    } else {
      if (iter->second.clientid == checkpoint.clientid) {
        iter->second.clientid.clear();
      }
    }
    iter->second.state = checkpoint.state;
  }
  publish_checkpoint_map();
}

void CheckpointServer::loadCheckPointCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<orient_interfaces::srv::LoadCheckPoint::Request> request,
    std::shared_ptr<orient_interfaces::srv::LoadCheckPoint::Response> response)
{
    std::stringstream ss(request->content);
    ss >> checkpoint_map_;
    resetCheckPoint();
    publish_checkpoint_map();
    publish_checkpoint_path();
    response->result = orient_interfaces::srv::LoadCheckPoint::Response::RESULT_SUCCESS;
}

void CheckpointServer::publish_checkpoint_path() {
  if (global_plan_publisher_->is_activated() && global_plan_publisher_->get_subscription_count() > 0) {
      std::unordered_set<orient_interfaces::msg::CheckPoint, orient_checkpoint::PointHasher, orient_checkpoint::PointEqual> checkpoints;
      for(const orient_interfaces::msg::CheckPointPath &path : checkpoint_map_.paths) {
        checkpoints.insert(path.points.begin(), path.points.end());
      }
      sensor_msgs::msg::PointCloud point_cloud;
      point_cloud.header.frame_id = global_frame_;
      point_cloud.header.stamp = now();
      for(const orient_interfaces::msg::CheckPoint &checkpoint:checkpoints) {
        geometry_msgs::msg::Point point;
        convertPoint(checkpoint, point);
        geometry_msgs::msg::Point32 point32;
        point32.x = static_cast<float>(point.x);
        point32.y = static_cast<float>(point.y);
        point32.z = static_cast<float>(point.z);
        point_cloud.points.push_back(point32);
      }
      global_plan_publisher_->publish(point_cloud);
  }
}

void CheckpointServer::resetCheckPoint() {
  for(orient_interfaces::msg::CheckPointPath &path : checkpoint_map_.paths) {
    path.points.erase(std::unique(path.points.begin(), path.points.end()), path.points.end());
  }

  checkpoint_map_.header.frame_id = global_frame_;
  set_global_checkpoints();
  setGraph();
}

bool CheckpointServer::loadMapFromYaml(const std::string & yaml_filename) {
   YAML::Node node = YAML::LoadFile(yaml_filename);
  if (!node) {
      return false;
  }

  auto image_file_name = node["image"].as<std::string>();
  if (image_file_name[0] != '/') {
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }

  Magick::InitializeMagick(nullptr);
  Magick::Image img(image_file_name);
  checkpoint_map_.info.resolution = node["resolution"].as<double>();
  checkpoint_map_.info.width = img.size().width();
  checkpoint_map_.info.height = img.size().height();
  YAML::Node origin_node = node["origin"];
  checkpoint_map_.info.origin.position.x = origin_node[0].as<double>();
  checkpoint_map_.info.origin.position.y = origin_node[1].as<double>();
  checkpoint_map_.info.origin.position.z = 0.0;
  checkpoint_map_.info.origin.orientation = nav2_util::geometry_utils::orientationAroundZAxis(origin_node[2].as<double>());

  double occupied_thresh = 0.65;
  if (node["occupied_thresh"]) {
    occupied_thresh = node["occupied_thresh"].as<double>();
  }
  double free_thresh = 0.196;
  if (node["free_thresh"]) {
    free_thresh = node["free_thresh"].as<double>();
  }
  bool negate = false;
  if (node["negate"]) { 
    negate = node["negate"].as<int>() != 0;
  } 

  nav_msgs::msg::OccupancyGrid map;
  map.info = checkpoint_map_.info;
  map.data.resize(map.info.width * map.info.height);
 for (size_t y = 0; y < map.info.height; y++) {
    for (size_t x = 0; x < map.info.width; x++) {
      auto pixel = img.pixelColor(x, y);
      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(), pixel.blueQuantum()};
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());
      double occ = (negate ? shade : 1.0 - shade);

      int8_t map_cell;
        if (occupied_thresh < occ) {
          map_cell = nav2_util::OCC_GRID_OCCUPIED;
        } else if (occ < free_thresh) {
          map_cell = nav2_util::OCC_GRID_FREE;
        } else {
          map_cell = nav2_util::OCC_GRID_UNKNOWN;
        }
      map.data[map.info.width * (map.info.height - y - 1) + x] = map_cell;
    }
  }

  costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(map);
  planner_ = std::make_unique<nav2_navfn_planner::NavFn>(costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());
  planner_->setCostmap(costmap_->getCharMap(), true, true);

  return true;
}

bool CheckpointServer::loadCheckPointFromYaml(const std::string &yaml_filename) {
  std::ifstream fin(yaml_filename);
  if (!fin) {
      return false;
  }
  fin >> checkpoint_map_;
  resetCheckPoint();
  return true;
}

std::optional<CheckpointServer::Vertex> CheckpointServer::getVertex(const orient_interfaces::msg::CheckPoint & point) {
  if (vertex_map_.find(point) == vertex_map_.end()) {
    return std::nullopt;
  }
  return vertex_map_[point];
}

void CheckpointServer::setGraph() {
  graph_.clear();
  vertex_map_.clear();
  
    // Add vertices
  for (const orient_interfaces::msg::CheckPoint& check_point : checkpoint_map_.points) {
      vertex_map_[check_point] = boost::add_vertex(check_point, graph_);
  }
  for (const CheckPointPath& path : checkpoint_map_.paths) {
      const auto &points = path.points;
      if (points.size() < 2 || getVertex(points[0]) == std::nullopt || getVertex(points[points.size() - 1]) == std::nullopt) {
        continue;
      }
      Vertex u = vertex_map_[points.front()];
      Vertex v = vertex_map_[points.back()];
      size_t weight = points.size();
      add_edge(u, v, weight, graph_);
  }
}

template<typename T>
bool CheckpointServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  RCLCPP_INFO(get_logger(), "goal->use_start: %d", goal->use_start);
  if (goal->use_start) {
    start = goal->start;
  } else if (!getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

bool  CheckpointServer::getRobotPose(geometry_msgs::msg::PoseStamped & start) {
  std::string tf_error;
  bool found = tf_->canTransform(robot_base_frame_, global_frame_, tf2::TimePointZero, &tf_error);
  if (!found) {
      return false;
  }
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_,transform_tolerance_)) {
      return false;
  }
  start = current_pose;
  return true;
}


void CheckpointServer::publish_current_checkpoint(const orient_interfaces::msg::CheckPointState &current_check_point) {
  orient_interfaces::msg::CheckPointStateStamp current_check_point_state;
  current_check_point_state.header.stamp = now();
  current_check_point_state.header.frame_id = global_frame_;
  current_check_point_state.checkpoint = current_check_point;
  current_checkpoint_publisher_->publish(current_check_point_state);
}

void CheckpointServer::set_global_checkpoints() {
  global_checkpoints_map_.clear();
  for(const orient_interfaces::msg::CheckPoint &checkpoint: checkpoint_map_.points) {
    orient_interfaces::msg::CheckPointState state;
    state.clientid = orient_clientid_;
    state.state = false;
    state.point = checkpoint;
    convertPoint(checkpoint, state.position);
    global_checkpoints_map_.insert(std::make_pair(checkpoint, state));
  }
}

std::vector<orient_interfaces::msg::CheckPointState> 
CheckpointServer::get_checkpoints_track(const std::vector<CheckPoint> &in_points) {
  std::vector<Vertex> predecessors(boost::num_vertices(graph_));
  std::vector<double> distances(boost::num_vertices(graph_));
  std::vector<orient_interfaces::msg::CheckPoint> check_points;
  if(in_points.size() ==1) {
    check_points = in_points;
  } else {
    for (size_t i = 0; i < in_points.size() - 1; i++) {
      Vertex start = vertex_map_[in_points[i]];
      Vertex goal = vertex_map_[in_points[i + 1]];
      int recursion = checkpoint_map_.paths.size();
      auto predecessor_map = boost::make_iterator_property_map(predecessors.begin(), get(boost::vertex_index, graph_));
      auto distance_map = boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, graph_));
      boost::dijkstra_shortest_paths(graph_, start, boost::predecessor_map(predecessor_map).distance_map(distance_map));

      std::vector<orient_interfaces::msg::CheckPoint> points;
      for (Vertex v = goal; v != start; v = predecessors[v]) {
          points.push_back(graph_[v]);
          if (recursion-- == 0) {
              points.clear();
              break;
          }
      }
      if (points.empty()) {
          check_points.clear();
          break;
      }
      points.push_back(graph_[start]);
      std::reverse(points.begin(), points.end());
      check_points.insert(check_points.end(), points.begin(), points.end());
    }
    check_points.erase(std::unique(check_points.begin(), check_points.end()), check_points.end());
  }


  std::vector<orient_interfaces::msg::CheckPointState> track;
  for (const auto &point : check_points) {
    auto iter = global_checkpoints_map_.find(point);
    if (iter != global_checkpoints_map_.end()) {
      track.push_back(iter->second);
    }
  }
  return track;
}

void CheckpointServer::mapToWorld(uint32_t mx, uint32_t my, double & wx, double & wy) const {
  wx = checkpoint_map_.info.origin.position.x+ (mx + 0.5) * checkpoint_map_.info.resolution;
  wy = checkpoint_map_.info.origin.position.y + (my + 0.5) * checkpoint_map_.info.resolution;
}

void CheckpointServer::worldToMap(double wx, double wy,uint32_t &  mx, uint32_t  & my) const {
  wx = std::round((wx - checkpoint_map_.info.origin.position.x) / checkpoint_map_.info.resolution);
  wy = std::round((wy - checkpoint_map_.info.origin.position.y) / checkpoint_map_.info.resolution);
  mx = static_cast<unsigned int>(wx);
  my = static_cast<unsigned int>(wy);
}

void CheckpointServer::get_checkpoint_map_callback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<orient_interfaces::srv::GetCheckPointMap::Request> request,
    std::shared_ptr<orient_interfaces::srv::GetCheckPointMap::Response> response)
{
  using namespace orient_interfaces::srv;
  response->result = GetCheckPointMap::Response::RESULT_INVALID_MAP_DATA;
  if (checkpoint_map_.points.size() == 0) {
    return;
  }
  response->map = checkpoint_map_;
  response->result = GetCheckPointMap::Response::RESULT_SUCCESS;
}

std::vector<int> CheckpointServer::getPathIndex(const geometry_msgs::msg::Point & robot_point) {
  std::vector<int> path_indices;
  for (size_t i = 0; i < checkpoint_map_.paths.size(); ++i) {
    const auto &path = checkpoint_map_.paths[i];
    if (path.points.empty()) {
      continue;
    }
    int nearset_index = getNearestPathPoint(robot_point, path.points);
    if (nearset_index != -1) {
        path_indices.push_back(i);
        break;  // No need to check other points in this path
    }
  }
  return path_indices;
}
int CheckpointServer::getNearestPathPoint(const geometry_msgs::msg::Point & point, const std::vector<orient_interfaces::msg::CheckPoint> & path_points) {
  int nearest_index = -1;
  for(int i = 0; i < static_cast<int>(path_points.size()); ++i) {
    const orient_interfaces::msg::CheckPoint &p = path_points[i];
    geometry_msgs::msg::Point pw;
    convertPoint(p, pw);
    double distance = nav2_util::geometry_utils::euclidean_distance(point, pw);
    if (distance < match_distance_thresh_) {
      nearest_index = static_cast<int>(i);
      break;  // No need to check other points in this path
    }
  }
  return nearest_index;
}

void CheckpointServer::appraise_checkpoint(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<orient_interfaces::srv::CheckPointPose::Request> request,
    std::shared_ptr<orient_interfaces::srv::CheckPointPose::Response> response)
{
  std::vector<orient_interfaces::msg::CheckPoint> in_checkpoints;
  for(const rmf_fleet_msgs::msg::Location &loc : request->path_request.path) {
    orient_interfaces::msg::CheckPoint checkpoint;
    if (get_checkpoint(loc, checkpoint)) {
      in_checkpoints.push_back(checkpoint);
    }
  }
  if (in_checkpoints.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid checkpoint in the request");
    return;
  }
  // Check if all points are in the map
  for (const orient_interfaces::msg::CheckPoint &point : in_checkpoints) {
    if (getVertex(point) == std::nullopt) {
      RCLCPP_ERROR(get_logger(), "Checkpoint %d %d not found in the map", point.x, point.y);
      return;
    }
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  getRobotPose(robot_pose);

  std::vector<std::pair<int, orient_interfaces::msg::CheckPoint>> start_checkpoints;
  if (current_check_point_.index == std::numeric_limits<uint32_t>::max()) {
    std::vector<int> path_indexs = getPathIndex(robot_pose.pose.position);
    if (path_indexs.empty()) {
      RCLCPP_WARN(get_logger(), "robot is not on path or any checkpoint found");
      return;
    }
    for(const int path_index : path_indexs) {
      orient_interfaces::msg::CheckPoint path_checkpoint = checkpoint_map_.paths[path_index].points.back();
      geometry_msgs::msg::Point point_in_world;
      convertPoint(path_checkpoint, point_in_world);
      geometry_msgs::msg::Point point_in_baselink  = orient_utils::transform_map_to_baselink(point_in_world, robot_pose.pose);
      if (point_in_baselink.x > 0.0 || allow_reversing_) {
        start_checkpoints.push_back(std::make_pair(path_index, path_checkpoint));
      }
    }
  } else {
    start_checkpoints.push_back(std::make_pair(-1, current_check_point_));
  }

  std::vector<orient_interfaces::msg::CheckPointState> track;
  int start_path = -1;
  for (const auto &start_checkpoint : start_checkpoints) {
    std::vector<CheckPoint> checkpoints = in_checkpoints;

    if (checkpoints.front() != start_checkpoint.second) {
      checkpoints.insert(checkpoints.begin(), start_checkpoint.second);
    }

    track = get_checkpoints_track(checkpoints);
    if (!track.empty()) {
      start_path = start_checkpoint.first;
      break;  // Found a valid track
    }
  }

  if (track.empty()) {
    RCLCPP_ERROR(get_logger(), "No checkpoint track found in the map");
    return;
  }
  RCLCPP_INFO(get_logger(), "Estimated track with %zu checkpoints", track.size());

  std::vector<geometry_msgs::msg::Point> points;
  for (const auto &checkpoint : track) {
    points.push_back(checkpoint.position);
  }

  response->poses = orient_utils::estimate_poses(points, robot_pose.pose);
  response->track = track;
  response->start_path = start_path;
}

bool CheckpointServer::get_checkpoint(const rmf_fleet_msgs::msg::Location &loc, orient_interfaces::msg::CheckPoint & checkpoint) {
  orient_interfaces::msg::CheckPoint point;
  point.x = static_cast<uint32_t>(loc.x);
  point.y = static_cast<uint32_t>(loc.y);
  point.z = 0.0;
  auto iter = global_checkpoints_map_.find(point);
  if (iter == global_checkpoints_map_.end()) {
    return false;
  }
  checkpoint = iter->second.point;
  return true;
}

bool CheckpointServer::get_checkpoint(const geometry_msgs::msg::Point & point, orient_interfaces::msg::CheckPoint & checkpoint) {
  auto checkpoint_opt = location_checkpoint(point);
  if (!checkpoint_opt.has_value()) {
    return false;
  }
  checkpoint = checkpoint_opt.value();
  return true;
}
std::optional<orient_interfaces::msg::CheckPoint> CheckpointServer::location_checkpoint(const geometry_msgs::msg::Point &position) {
  for (const auto [point,s] : global_checkpoints_map_) {
    double d = nav2_util::geometry_utils::euclidean_distance(s.position, position);
    if (d < match_distance_thresh_) {
      return std::make_optional(s.point);
    }
  }
  return std::nullopt;
}

void CheckpointServer::publish_checkpoint_map() {
  orient_interfaces::msg::CheckPointStateMap checkpoint_state_map_msg;
  checkpoint_state_map_msg.header.stamp = now();
  checkpoint_state_map_msg.header.frame_id = global_frame_;
  for(const auto &checkpoint_pair : global_checkpoints_map_) {
    checkpoint_state_map_msg.checkpoints.push_back(checkpoint_pair.second);
  }
  checkpoint_publisher_->publish(checkpoint_state_map_msg);
}

template<typename T>
void CheckpointServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

std::vector<geometry_msgs::msg::Pose> CheckpointServer::make_path(
  const geometry_msgs::msg::PoseStamped &start,
  std::vector<geometry_msgs::msg::PoseStamped> goals, bool backup) {
  std::vector<geometry_msgs::msg::Point> points;
  if (goals.empty()) {
    return {};
  }
  auto goal_start = goals.front().pose.position;

  //如果开始pose和goal的pose相同，且z轴不为-1，则goal_start.z表示为robot的行进开始路径
  if (goal_start.x == goal_start.y && goal_start.x == -1 && goal_start.z != -1) {
    const auto &start_path = checkpoint_map_.paths[goal_start.z];
    int nearest_index = getNearestPathPoint(start.pose.position, start_path.points);
    if (nearest_index != -1) {
      std::transform(std::next(start_path.points.begin(), nearest_index),start_path.points.end() , std::back_inserter(points),
        [this](const orient_interfaces::msg::CheckPoint &p) {
          geometry_msgs::msg::Point point;
          convertPoint(p, point);
          return point;
        });

      RCLCPP_INFO(get_logger(), "Start pose is on a valid path, using it as start. %d", nearest_index);
    } else {
      RCLCPP_WARN(get_logger(), "Start pose is not on a valid path, using first goal as start.");
    }
    goals.erase(goals.begin());
  }

  for(size_t i = 0; i < goals.size() - 1; ++i) {
    auto curr_path_points = getPlanPoints(goals[i].pose.position, goals[i + 1].pose.position);
    points.insert(points.end(), curr_path_points.begin(), curr_path_points.end());
  }
  points.erase(std::unique(points.begin(), points.end()), points.end());
  return orient_utils::estimate_poses(points,start.pose, backup);
}
static inline double squared_distance( const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return dx * dx + dy * dy;
}

std::vector<geometry_msgs::msg::Pose> 
CheckpointServer::make_path(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped & goal) {
  std::vector<geometry_msgs::msg::Pose> poses;

  double distance = nav2_util::geometry_utils::euclidean_distance(start.pose.position, goal.pose.position);
  if (distance < match_distance_thresh_) {
    unsigned int mx, my;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(get_logger(), "Failed to create a unique pose path because of obstacles");
      return poses;
    }
    geometry_msgs::msg::Pose pose;
    pose = start.pose;
    if (start.pose.orientation != goal.pose.orientation) {
      pose.orientation = goal.pose.orientation;
    }
    poses.push_back(pose);
    return poses;
  }

  double resolution = costmap_->getResolution();
  uint32_t mx,my;
  int map_start[2];
  worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  map_start[0] = mx;
  map_start[1] = my;

  int map_goal[2];
  worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  planner_->calcNavFnDijkstra(true);
  
  bool found_legal = false;
  geometry_msgs::msg::Pose p, best_pose;
  double tolerance = 0.5;

  p = goal.pose;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    best_pose = p;
    found_legal = true;
  } else {
    double best_sdist = std::numeric_limits<double>::max();
    p.position.y = goal.pose.position.y - tolerance;
    while (p.position.y <= goal.pose.position.y + tolerance) {
      p.position.x = goal.pose.position.x - tolerance;
      while (p.position.x <= goal.pose.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal.pose);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  if (found_legal) {
      // extract the plan
    getPlanFromPotential(best_pose, poses);
  }
  return poses;
}

bool
CheckpointServer::getPlanFromPotential(const geometry_msgs::msg::Pose & goal, std::vector<geometry_msgs::msg::Pose> & poses)
{
  // the potential has already been computed, so we won't update our copy of the costmap
  int map_goal[2];
  uint32_t my, mx;
  worldToMap(goal.position.x, goal.position.y, mx, my);
  map_goal[0] = mx;
  map_goal[1] = my;
  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::Pose pose;
    pose.position.x = world_x;
    pose.position.y = world_y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    poses.push_back(pose);
  }

  return !poses.empty();
}
double
CheckpointServer::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  worldToMap(world_point.x, world_point.y, mx, my);
  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}
void
CheckpointServer::computePlanThroughPoses()
{
  RCLCPP_INFO(get_logger(), "Computing plan through poses");
  if (action_server_poses_->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server_poses_->terminate_all();
    return;
  }
  auto goal = action_server_poses_->get_current_goal();
  auto start_time = this->now();

  auto result = std::make_shared<ActionThroughPoses::Result>();

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      action_server_poses_->terminate_current();
      return;
    }
    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.size() == 0) {
      RCLCPP_WARN(get_logger(), "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_poses_->terminate_current();
      return;
    }
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      RCLCPP_WARN(get_logger(), "Failed to get start pose, returning.");
      action_server_poses_->terminate_current();
      return;
    }
    auto goals = goal->goals;

    std::vector<geometry_msgs::msg::Pose> poses;
    RCLCPP_WARN(get_logger(), "Computing path through %zu poses.", goals.size());

    if (goals.size() == 1) {
      poses = make_path(start, goals.front());

    } else {
      poses = make_path(start, goals, goal->planner_id == "Backup");
    }
    nav_msgs::msg::Path concat_path;
    concat_path.header.frame_id = global_frame_;
    concat_path.header.stamp = now();
    concat_path.poses.reserve(poses.size());
    for (const auto &pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = concat_path.header;
      pose_stamped.pose = pose;
      concat_path.poses.push_back(pose_stamped);
    }
    smooth(concat_path, goals);

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(concat_path);
    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    action_server_poses_->terminate_current();
  }
}


int CheckpointServer::getCheckPointPath(const orient_interfaces::msg::CheckPoint &start, const orient_interfaces::msg::CheckPoint &end) {
  int idx  = -1;
  for(size_t i = 0; i < checkpoint_map_.paths.size(); ++i) {
    const CheckPointPath &path = checkpoint_map_.paths[i];
    if (path.points.front() == start &&  path.points.back() == end) {
      idx = static_cast<int>(i);
      break;
    }
  } return idx;    
}

std::vector<geometry_msgs::msg::Point>
CheckpointServer::getPlanPoints(const geometry_msgs::msg::Point & start_position,const geometry_msgs::msg::Point & goal_position) {
  std::vector<geometry_msgs::msg::Point> points;
  // If the start and goal are the same, return an empty path
  orient_interfaces::msg::CheckPoint start_checkpoint, goal_checkpoint;
  if(get_checkpoint(start_position,start_checkpoint) && get_checkpoint(goal_position, goal_checkpoint)) {
    int path_index = getCheckPointPath(start_checkpoint, goal_checkpoint);
    if (path_index != -1) {
      const CheckPointPath &path = checkpoint_map_.paths[path_index];
      for(const auto &point : path.points) {
        geometry_msgs::msg::Point p;
        convertPoint(point, p);
        points.push_back(p);
      }
    }
  }
  if (points.empty()) {
    // If no path found, use a straight line between start and goal
    points = orient_utils::getStraightLinePoints(start_position, goal_position, interpolation_resolution_);
  }
  if (points.size() < 2) {
    points = {start_position,goal_position};
  }
  return points;
}

void
CheckpointServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

bool CheckpointServer::smooth(nav_msgs::msg::Path &path,const std::vector<geometry_msgs::msg::PoseStamped> &goals, const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 3) {
    return true;
  }

  // 记录goals在path中的索引，保证这些点不会被平滑算法修改
  std::vector<size_t> goal_indices;
  for (const auto &goal : goals) {
    for (size_t i = 0; i < path.poses.size(); ++i) {
      if (std::hypot(
            path.poses[i].pose.position.x - goal.pose.position.x,
            path.poses[i].pose.position.y - goal.pose.position.y) < 1e-6) {
        goal_indices.push_back(i);
        break;
      }
    }
  }

  // 排序，方便后续查找
  std::sort(goal_indices.begin(), goal_indices.end());

  // 标记哪些点是goal点
  std::vector<bool> is_goal(path.poses.size(), false);
  for (auto idx : goal_indices) {
    if (idx < is_goal.size()) {
      is_goal[idx] = true;
    }
  }

  // 平滑时跳过goal点
  auto is_fixed = [&](size_t idx) {
    return idx == 0 || idx == path.poses.size() - 1 || is_goal[idx];
  };

  refinement_ctr_ = 0;
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 9) {
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();

      // 只平滑非goal点
      int its = 0;
      double change = tolerance_;
      nav_msgs::msg::Path new_path = curr_path_segment;
      nav_msgs::msg::Path last_path = curr_path_segment;
      const unsigned int & path_size = curr_path_segment.poses.size();

      while (change >= tolerance_) {
        its += 1;
        change = 0.0;
        if (its >= max_its_) {
          curr_path_segment = last_path;
          smoother_utils::updateApproximatePathOrientations(curr_path_segment, reversing_segment);
          success = false;
          break;
        }
        steady_clock::time_point b = steady_clock::now();
        rclcpp::Duration timespan(duration_cast<duration<double>>(b - now));
        if (timespan > rclcpp::Duration::from_seconds(time_remaining)) {
          curr_path_segment = last_path;
          smoother_utils::updateApproximatePathOrientations(curr_path_segment, reversing_segment);
          success = false;
          break;
        }
        for (unsigned int j = 1; j != path_size - 1; j++) {
          // 跳过goal点
          if (is_fixed(j + path_segments[i].start)) {
            continue;
          } 

          for (unsigned int k = 0; k != 2; k++) {
            double x_i = getFieldByDim(curr_path_segment.poses[j], k);
            double y_i = getFieldByDim(new_path.poses[j], k);
            double y_m1 = getFieldByDim(new_path.poses[j - 1], k);
            double y_ip1 = getFieldByDim(new_path.poses[j + 1], k);
            double y_i_org = y_i;
            y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
            setFieldByDim(new_path.poses[j], k, y_i);
            change += abs(y_i - y_i_org);
          }
        }
        last_path = new_path;
      }
      if (do_refinement_ && refinement_ctr_ < 4) {
        refinement_ctr_++;
        // refinement时也跳过goal点
        // 递归调用略去，实际可仿照上面逻辑
      }
      smoother_utils::updateApproximatePathOrientations(new_path, reversing_segment);
      smooth(new_path, reversing_segment);
      curr_path_segment = new_path;

      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  for(size_t i = 1; i < path.poses.size() -1; ++i) {
    if (is_fixed(i)) {
      auto &prev = path.poses[i - 1].pose.orientation;
      auto &next = path.poses[i + 1].pose.orientation;
      // 计算平均方向
      double yaw_prev = tf2::getYaw(prev);
      double yaw_next = tf2::getYaw(next);
      double avg_yaw = std::atan2(std::sin(yaw_prev) + std::sin(yaw_next), std::cos(yaw_prev) + std::cos(yaw_next));
      path.poses[i].pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(avg_yaw / 2), std::cos(avg_yaw / 2))); 
      
      // path.poses[i].pose.orientation = path.poses[i + 1].pose.orientation;
    }
  }
  path.poses.back().pose.orientation = path.poses[path.poses.size() -2].pose.orientation;
  path.poses.front().pose.orientation = path.poses[1].pose.orientation;

  return success;
}

bool CheckpointServer::smooth(nav_msgs::msg::Path & path,bool & reversing_segment)
{
  // Must be at least 10 in length to enter function
  const unsigned int & path_size = path.poses.size();

  // 7-point SG filter
  const std::array<double, 7> filter = {
    -2.0 / 21.0,
    3.0 / 21.0,
    6.0 / 21.0,
    7.0 / 21.0,
    6.0 / 21.0,
    3.0 / 21.0,
    -2.0 / 21.0};

  auto applyFilter = [&](const std::vector<geometry_msgs::msg::Point> & data)
    -> geometry_msgs::msg::Point
    {
      geometry_msgs::msg::Point val;
      for (unsigned int i = 0; i != filter.size(); i++) {
        val.x += filter[i] * data[i].x;
        val.y += filter[i] * data[i].y;
      }
      return val;
    };

  auto applyFilterOverAxes =
    [&](std::vector<geometry_msgs::msg::PoseStamped> & plan_pts) -> void
    {
      // Handle initial boundary conditions, first point is fixed
      unsigned int idx = 1;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      // Apply nominal filter
      for (idx = 3; idx < path_size - 4; ++idx) {
        plan_pts[idx].pose.position = applyFilter(
        {
          plan_pts[idx - 3].pose.position,
          plan_pts[idx - 2].pose.position,
          plan_pts[idx - 1].pose.position,
          plan_pts[idx].pose.position,
          plan_pts[idx + 1].pose.position,
          plan_pts[idx + 2].pose.position,
          plan_pts[idx + 3].pose.position});
      }

      // Handle terminal boundary conditions, last point is fixed
      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 2].pose.position});

      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position});
    };

  applyFilterOverAxes(path.poses);

  // Lets do additional refinement, it shouldn't take more than a couple milliseconds
  if (do_refinement_) {
    for (int i = 0; i < refinement_num_; i++) {
      applyFilterOverAxes(path.poses);
    }
  }

  updateApproximatePathOrientations(path, reversing_segment);
  return true;
}
double CheckpointServer::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

void CheckpointServer::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

template<typename T>
bool CheckpointServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

template<typename T>
bool CheckpointServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(orient_checkpoint::CheckpointServer)
