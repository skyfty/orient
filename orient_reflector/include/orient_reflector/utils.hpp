#ifndef ORIENT_REFECTOR_UTILS_H
#define ORIENT_REFECTOR_UTILS_H

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "orient_interfaces/msg/reflector_grid.hpp" // CHANGE
#include "orient_interfaces/msg/reflector_map.hpp" // CHANGE
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "yaml-cpp/yaml.h"

#include "tf2_ros/buffer.h"

namespace orient_reflector{

 inline geometry_msgs::msg::Point operator-(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    geometry_msgs::msg::Point r;
    r.x = p1.x - p2.x;
    r.y = p1.y - p2.y;
    return  r;
}

 inline geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
    geometry_msgs::msg::Point r;
    r.x = p1.x + p2.x;
    r.y = p1.y + p2.y;
    return  r;
}

std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, tf2_ros::Buffer* tf_buffer);
    
double euclidean_distance(const orient_interfaces::msg::Reflector &p1, const orient_interfaces::msg::Reflector &p2);

std::tuple<double,double, double, double> get_reflector_round_bounds(const std::vector<orient_interfaces::msg::Reflector> &reflectors);
geometry_msgs::msg::Pose trilateration_from_reflector(const std::vector<orient_interfaces::msg::Reflector> &reflectors_local);


bool saveReflector(const std::string & yaml_file, const orient_interfaces::msg::ReflectorMap & map);
std::ostream & operator<<(std::ostream & os, const orient_interfaces::msg::ReflectorMap & map);
YAML::Node & operator<<(YAML::Node &e, const orient_interfaces::msg::ReflectorMap & reflector_map);
std::istream & operator>>(std::istream & is, orient_interfaces::msg::ReflectorMap & map);
}
#endif