#include <cstdio>
#include <string>
#include <fstream>
#include <optional>
#include <boost/smart_ptr.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "orient_reflector/utils.hpp"
#include "Eigen/Dense"
#include "orient_reflector/urdf/model.h"

namespace orient_reflector{
    
YAML::Node & operator<<(YAML::Node &e, const orient_interfaces::msg::ReflectorMap & reflector_map) {
    
    e["origin"] = YAML::Load("[" + std::to_string(reflector_map.origin.position.x) + ", " + std::to_string(reflector_map.origin.position.y) + ", 0.0]");

    YAML::Node     reflectors_nodes = YAML::Node(YAML::NodeType::Sequence);
    for (const orient_interfaces::msg::Reflector &reflector: reflector_map.reflectors) {
        YAML::Node reflector_node;
        reflector_node["id"] = reflector.id;
        reflector_node["diameter"] = reflector.diameter;
        reflector_node["intensity"] = reflector.intensity;
        reflector_node["x"] = reflector.point.x;    
        reflector_node["y"] = reflector.point.y;
        reflectors_nodes.push_back(reflector_node);
    }
    e["reflectors"] = reflectors_nodes;

    return e;
}
std::ostream & operator<<(std::ostream &os, const orient_interfaces::msg::ReflectorMap & reflector_map) {

    YAML::Node reflector_map_node;
    reflector_map_node << reflector_map;
    YAML::Emitter e;
    e << reflector_map_node;
    os << e.c_str();
    return os;
}


std::istream & operator>>(std::istream &is, orient_interfaces::msg::ReflectorMap& reflector_map) {
    YAML::Node reflector_grid_node = YAML::Load(is);  
    if (!reflector_grid_node) {  
        return is;
    }
    YAML::Node origin_node = reflector_grid_node["origin"];
    reflector_map.origin.position.x = origin_node[0].as<double>();
    reflector_map.origin.position.y = origin_node[1].as<double>();
    reflector_map.origin.position.z = 0.;
    reflector_map.origin.orientation.w = 1.;
    reflector_map.origin.orientation.x = 0.;
    reflector_map.origin.orientation.y = 0.;
    reflector_map.origin.orientation.z = 0.;
    reflector_map.reflectors.clear();
    YAML::Node reflectors_node = reflector_grid_node["reflectors"].as<YAML::Node>();
    for (const auto& reflector_node : reflectors_node) {  
        orient_interfaces::msg::Reflector reflector;
        reflector.id = reflector_node["id"].as<int>();
        reflector.intensity = reflector_node["intensity"].as<double>();
        reflector.diameter = reflector_node["diameter"].as<double>();
        reflector.point.x = reflector_node["x"].as<double>();
        reflector.point.y = reflector_node["y"].as<double>();
        reflector_map.reflectors.push_back(reflector);
    }
    return is;
}

std::tuple<double,double, double, double> get_reflector_round_bounds(const std::vector<orient_interfaces::msg::Reflector> &reflectors) {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (const auto& reflector : reflectors) {
        min_x = std::min(min_x, reflector.point.x);
        max_x = std::max(max_x, reflector.point.x);
        min_y = std::min(min_y, reflector.point.y);
        max_y = std::max(max_y, reflector.point.y);
    }
    return std::make_tuple(min_x, max_x, min_y, max_y);
}


geometry_msgs::msg::Pose trilateration_from_reflector(const std::vector<orient_interfaces::msg::Reflector> &reflectors_local) {
    // 创建两个动态大小的矩阵，分别存储局部坐标
    Eigen::Matrix<double, 2, Eigen::Dynamic> locals(2, reflectors_local.size());

    // 将匹配的点的局部坐标填入矩阵
    for (size_t i = 0; i < reflectors_local.size(); i++) {
        locals(0, i) = reflectors_local[i].point.x;
        locals(1, i) = reflectors_local[i].point.y;
    }

    // 计算局部坐标的质心
    Eigen::Vector2d centroid = locals.rowwise().mean();
    geometry_msgs::msg::Pose pose;
    pose.position.x = centroid(0);
    pose.position.y = centroid(1);
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0; 
    pose.orientation.w = 1; 
    return pose;
}

double euclidean_distance(const orient_interfaces::msg::Reflector &p1, const orient_interfaces::msg::Reflector &p2) {
    return nav2_util::geometry_utils::euclidean_distance(p1.point, p2.point);
}


std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, tf2_ros::Buffer* const tf_buffer) {
  urdf::Model model;
  model.initFile(urdf_filename);
  std::vector<std::shared_ptr<urdf::Link> > links;
  model.getLinks(links);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  for (const auto& link : links) {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED) {
      continue;
    }

    const urdf::Pose& pose = link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation.w = pose.rotation.w;
    transform.transform.rotation.x = pose.rotation.x;
    transform.transform.rotation.y = pose.rotation.y;
    transform.transform.rotation.z = pose.rotation.z;
    transform.child_frame_id = link->name;
    transform.header.frame_id = link->getParent()->name;
    tf_buffer->setTransform(transform, "urdf", true /* is_static */);
    transforms.push_back(transform);
  }
  return transforms;
}

}