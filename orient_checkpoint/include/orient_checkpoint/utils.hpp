#ifndef CHECKPOINT_HELPER_HPP
#define CHECKPOINT_HELPER_HPP

#include "orient_interfaces/msg/check_point_map.hpp"             // CHANGE

namespace orient_checkpoint {

struct PointHasher {
    std::size_t operator()(const orient_interfaces::msg::CheckPoint &point) const {
        return std::hash<float>()(point.x) ^ std::hash<float>()(point.y) ^ std::hash<float>()(point.z);
    }
};

struct PointEqual {
    bool operator()(const orient_interfaces::msg::CheckPoint &a, const orient_interfaces::msg::CheckPoint &b) const {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};
void save_yaml(const std::string & yaml_file, const orient_interfaces::msg::CheckPointMap &checkpoint_map);
inline float invert_y_axis(uint32_t y) {
    return static_cast<float>(-(y + 0.001)); // Invert y-axis and add a small offset to avoid precision loss
}

std::ostream& operator<<(std::ostream& os, const orient_interfaces::msg::CheckPointMap &checkpoint_map);
std::istream& operator>>(std::istream& is, orient_interfaces::msg::CheckPointMap &checkpoint_map);
}
#endif // CHECKPOINT_HELPER_HPP
