#include "orient_checkpoint/utils.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "nav2_util/geometry_utils.hpp"

#include "orient_utils/estimate.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace orient_checkpoint {

std::ostream& operator<<(std::ostream& os, const orient_interfaces::msg::CheckPointMap &checkpoint_map) {
    YAML::Emitter e;
    e << YAML::BeginMap;

    e << YAML::Key << "building_name" << YAML::Value << checkpoint_map.building_name;
    e << YAML::Key << "levels" << YAML::Value << YAML::BeginMap;
    e << YAML::Key << checkpoint_map.level << YAML::Value << YAML::BeginMap;

    e << YAML::Key << "vertices" << YAML::Value << YAML::BeginSeq;
    for (const orient_interfaces::msg::CheckPoint &check_point : checkpoint_map.points) {
        e << YAML::BeginSeq;
        
        float wx = check_point.x + 0.001;  // to avoid precision loss during conversion
        float wy = orient_checkpoint::invert_y_axis(check_point.y);
        e << wx;
        e << wy;
        e << YAML::BeginMap;
        e << YAML::Key << "name" << YAML::Value << check_point.name;
        e << YAML::Key << "level" << YAML::Value << check_point.level;
        e << YAML::Key << "is_charger" << YAML::Value << check_point.is_charger;
        e << YAML::Key << "is_holding_point" << YAML::Value << check_point.is_holding_point;
        e << YAML::Key << "is_parking_spot" << YAML::Value << check_point.is_parking_spot;
        e << YAML::Key << "is_passthrough_point" << YAML::Value << check_point.is_passthrough_point;
        e << YAML::Key << "dock_name" << YAML::Value << check_point.dock_name;
        e << YAML::EndMap;
        e << YAML::EndSeq;
    }
    e << YAML::EndSeq;

    e << YAML::Key << "lanes" << YAML::Value << YAML::BeginSeq;
    for (const orient_interfaces::msg::CheckPointPath &point_path : checkpoint_map.paths) {
        e << YAML::BeginSeq;
        e << point_path.start;
        e << point_path.end;
        e << YAML::BeginMap;
        e << YAML::Key << "speed_limit" << YAML::Value << point_path.speed_limit;
        e << YAML::Key << "is_bidirectional" << YAML::Value << point_path.is_bidirectional;
        e << YAML::Key << "orientation_constraint" << YAML::Value << point_path.orientation_constraint;
        e << YAML::EndMap;
        e << YAML::BeginSeq;
        for (const orient_interfaces::msg::CheckPoint &point : point_path.points) {
            e << YAML::BeginSeq;
            e << point.x;
            e << point.y;
            e << YAML::EndSeq;
        }
        e << YAML::EndSeq;
        e << YAML::EndSeq;
    }
    e << YAML::EndSeq;

    e << YAML::EndMap;

    os << e.c_str();
    return os;
}

static std::vector<orient_interfaces::msg::CheckPoint> bresenhamLine(const orient_interfaces::msg::CheckPoint &p1, const orient_interfaces::msg::CheckPoint &p2) {
    int x1 = p1.x, y1 = p1.y;
    int x2 = p2.x, y2 = p2.y;

    // 计算绝对差值和步进方向
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    
    // 初始化误差和点列表
    int err = dx - dy;
    std::vector<orient_interfaces::msg::CheckPoint> points;
    
    // 主循环生成路径点
    while (true) {
        orient_interfaces::msg::CheckPoint pt;
        pt.x = x1;
        pt.y = y1;
        pt.index = std::numeric_limits<uint32_t>::max();
        pt.is_charger = false;
        pt.is_holding_point = false;
        pt.is_parking_spot = false;
        pt.is_passthrough_point = true;

        points.push_back(pt);

        // 到达终点时退出循环
        if (x1 == x2 && y1 == y2) 
            break;
        
        int e2 = 2 * err;
        
        // 调整误差并移动坐标
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    points.erase(points.begin());
    points.pop_back(); // Remove the start and end points
    
    return points;
}


std::istream& operator>>(std::istream& is, orient_interfaces::msg::CheckPointMap &checkpoint_map) {
    YAML::Node node = YAML::Load(is);
    if (!node) {
        return is;
    }
    checkpoint_map.building_name = node["building_name"].as<std::string>();
    YAML::Node levels_node = node["levels"];
    if (!levels_node) {
        return is;
    }

    for (const auto &level_node_pair : levels_node) {
        std::string key = level_node_pair.first.as<std::string>();
        checkpoint_map.level = key;
        YAML::Node level_node = level_node_pair.second;
        YAML::Node vertices_node = level_node["vertices"];
        if (!vertices_node || vertices_node.size() == 0) {
           continue;
        }

        for (size_t index = 0; index < vertices_node.size(); ++index) {
            const auto &vertex_node = vertices_node[index];
            orient_interfaces::msg::CheckPoint check_point;
            check_point.level = checkpoint_map.level;
            check_point.index = index;
            check_point.x = static_cast<uint32_t>(vertex_node[0].as<float>());
            check_point.y = static_cast<uint32_t>(-vertex_node[1].as<float>());
            check_point.z = 0; // Assuming z is always 0 for 2D
            YAML::Node prop_node = vertex_node[2];
            if (prop_node["name"]) {
                check_point.name = prop_node["name"].as<std::string>();
            }
            
            if (prop_node["is_charger"]) {
                check_point.is_charger = prop_node["is_charger"].as<bool>();
            } else {
                check_point.is_charger = false;
            }
            if (prop_node["is_holding_point"]) {
                check_point.is_holding_point = prop_node["is_holding_point"].as<bool>();
            } else {
                check_point.is_holding_point = false;
            }  
            if (prop_node["is_parking_spot"]) {
                check_point.is_parking_spot = prop_node["is_parking_spot"].as<bool>();
            } else {
                check_point.is_parking_spot = false;
            }
            if (prop_node["is_passthrough_point"]) {
                check_point.is_passthrough_point = prop_node["is_passthrough_point"].as<bool>();
            } else {
                check_point.is_passthrough_point = false;
            }
            if (prop_node["dock_name"]) {
                check_point.dock_name = prop_node["dock_name"].as<std::string>();
            }
            checkpoint_map.points.push_back(check_point);
        }

        YAML::Node lanes_node = level_node["lanes"];
        if (lanes_node) {
            for (const auto &lane_node : lanes_node) {
                orient_interfaces::msg::CheckPointPath point_path;
                point_path.start = lane_node[0].as<uint32_t>();
                point_path.end = lane_node[1].as<uint32_t>();
                YAML::Node prop_node = lane_node[2];
                if (prop_node["speed_limit"]) {
                    point_path.speed_limit = prop_node["speed_limit"].as<float>();
                }
                if (prop_node["is_bidirectional"]) {
                    point_path.is_bidirectional = prop_node["is_bidirectional"].as<bool>();
                }
                if (prop_node["orientation_constraint"]) {
                    point_path.orientation_constraint = prop_node["orientation_constraint"].as<std::string>();
                }
                YAML::Node path_node = lane_node[3];
                if (path_node) {
                    for (const auto &point_node : path_node) {
                        orient_interfaces::msg::CheckPoint point;
                        point.level = checkpoint_map.level;
                        point.index = std::numeric_limits<uint32_t>::max(); // Not set for path points
                        point.is_charger = false;
                        point.is_holding_point = false;
                        point.is_parking_spot = false;
                        point.is_passthrough_point = true;
                        point.x = point_node[0].as<uint32_t>();
                        point.y = point_node[1].as<uint32_t>();
                        point.z = 0; // Assuming z is always 0 for 2D
                        point_path.points.push_back(point);
                    }
                } else {
                    auto start_point = checkpoint_map.points[point_path.start];
                    auto end_point = checkpoint_map.points[point_path.end];
                    point_path.points.push_back(start_point);
                    std::vector<orient_interfaces::msg::CheckPoint> points = bresenhamLine(start_point, end_point);
                    point_path.points.insert(point_path.points.end(), points.begin(), points.end());
                    point_path.points.push_back(end_point);
                }
                checkpoint_map.paths.push_back(point_path);
            }
        }
        break;
    }
    return is;
}

void save_yaml(const std::string & yaml_filename, const orient_interfaces::msg::CheckPointMap &checkpoint_map) {
    std::ofstream fout(yaml_filename);
    fout << checkpoint_map;
    fout.close();
}

}
