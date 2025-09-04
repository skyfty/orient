#include "orient_reflector/probability_grid.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <nav2_util/occ_grid_values.hpp>

namespace orient_reflector {

ProbabilityGrid::ProbabilityGrid(double resolution, int width, int height, 
                                 double origin_x, double origin_y)
    : resolution_(resolution), width_(width), height_(height),
      origin_x_(origin_x), origin_y_(origin_y) {
    // 初始化地图为未知状态
    grid_data_.resize(width * height, nav2_util::OCC_GRID_UNKNOWN);
}

void ProbabilityGrid::worldToMap(double wx, double wy, int& mx, int& my) const {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);
}

void ProbabilityGrid::mapToWorld(int mx, int my, double& wx, double& wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool ProbabilityGrid::isValidMapPoint(int mx, int my) const {
    return mx >= 0 && mx < width_ && my >= 0 && my < height_;
}

void ProbabilityGrid::updateCell(int x, int y, bool hit) {
    if (!isValidMapPoint(x, y)) {
        return;
    }
    
    int index = y * width_ + x;
    int8_t current_val = grid_data_[index];
    double current_prob;
    
    // 初始化未知栅格
    if (current_val == nav2_util::OCC_GRID_UNKNOWN) {
        current_prob = 0.5;  // 未知状态设为50%概率
    } else {
        current_prob = current_val / 100.0;
    }
    
    // 更新概率
    double new_prob;
    if (hit) {
        new_prob = current_prob * hit_prob_;
    } else {
        new_prob = current_prob * (1 - miss_prob_);
    }
    
    // 限制概率范围
    new_prob = std::max(std::min(new_prob, max_prob_), min_prob_);
    
    // 转换为0-100的整数值
    grid_data_[index] = hit?nav2_util::OCC_GRID_OCCUPIED:nav2_util::OCC_GRID_FREE;
}

void ProbabilityGrid::setCellValue(int x, int y, int8_t value) {
    if (isValidMapPoint(x, y)) {
        grid_data_[y * width_ + x] = value;
    }
}

int8_t ProbabilityGrid::getCellValue(int x, int y) const {
    if (isValidMapPoint(x, y)) {
        return grid_data_[y * width_ + x];
    }
    return nav2_util::OCC_GRID_UNKNOWN;
}

void ProbabilityGrid::updateFromScan(const Eigen::Vector2d& robot_position, 
                                     const std::vector<Eigen::Vector2d>& scan_points) {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    // 获取机器人位置在地图坐标
    int rx, ry;
    worldToMap(robot_position.x(), robot_position.y(), rx, ry);
    
    // 更新每个激光点
    for (const auto& point : scan_points) {
        // 转换目标点到地图坐标
        int px, py;
        worldToMap(point.x(), point.y(), px, py);
        
        // 使用Bresenham算法获取激光路径上的所有栅格
        auto path = bresenhamLine(rx, ry, px, py);
        
        // 更新路径上的栅格 (除最后一个外都是未击中)
        for (size_t i = 0; i < path.size(); ++i) {
            const auto& cx = path[i].x();
            const auto& cy = path[i].y();
            
            if (isValidMapPoint(cx, cy)) {
                if (i == path.size() - 1) {  // 最后一个点是击中点
                    updateCell(cx, cy, true);
                } else {  // 路径上的其他点是未击中
                    updateCell(cx, cy, false);
                }
            }
        }
    }
}

std::vector<Eigen::Vector2i> ProbabilityGrid::bresenhamLine(int x0, int y0, int x1, int y1) const {
    std::vector<Eigen::Vector2i> points;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        points.emplace_back(x0, y0);
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return points;
}
void ProbabilityGrid::expandIfNeeded(double robot_x, double robot_y) {
    const double expansion_threshold = 8.0; // 当机器人距离边界0.5米时扩展
    
    double map_min_x = origin_x_;
    double map_max_x = origin_x_ + width_ * resolution_;
    double map_min_y = origin_y_;
    double map_max_y = origin_y_ + height_ * resolution_;
    
    bool need_expand = false;
    double new_origin_x = origin_x_;
    double new_origin_y = origin_y_;
    int new_width = width_;
    int new_height = height_;
    
    // 检查是否需要扩展
    if (robot_x - map_min_x < expansion_threshold) {
        new_origin_x -= resolution_ * 50; // 向左扩展50像素
        new_width += 50;
        need_expand = true;
    } else if (map_max_x - robot_x < expansion_threshold) {
        new_width += 50; // 向右扩展50像素
        need_expand = true;
    }
    
    if (robot_y - map_min_y < expansion_threshold) {
        new_origin_y -= resolution_ * 50; // 向下扩展50像素
        new_height += 50;
        need_expand = true;
    } else if (map_max_y - robot_y < expansion_threshold) {
        new_height += 50; // 向上扩展50像素
        need_expand = true;
    }
    
    // 如果需要扩展，创建新地图
    if (need_expand) {
        expandGrid(new_width, new_height, new_origin_x, new_origin_y);
    }
}

void ProbabilityGrid::expandGrid(int new_width, int new_height, 
                                 double new_origin_x, double new_origin_y) {
    // 创建新地图
    std::vector<int8_t> new_grid(new_width * new_height, nav2_util::OCC_GRID_UNKNOWN);
    
    // 计算原地图在新地图中的位置偏移
    int offset_x = static_cast<int>((origin_x_ - new_origin_x) / resolution_);
    int offset_y = static_cast<int>((origin_y_ - new_origin_y) / resolution_);
    
    // 复制原地图数据到新地图
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int new_x = x + offset_x;
            int new_y = y + offset_y;
            
            if (new_x >= 0 && new_x < new_width && new_y >= 0 && new_y < new_height) {
                new_grid[new_y * new_width + new_x] = grid_data_[y * width_ + x];
            }
        }
    }
    
    // 更新地图
    grid_data_ = std::move(new_grid);
    width_ = new_width;
    height_ = new_height;
    origin_x_ = new_origin_x;
    origin_y_ = new_origin_y;
}
void ProbabilityGrid::reinforceReflectors(const std::vector<orient_interfaces::msg::Reflector>& reflectors) {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    for (const auto& reflector : reflectors) {
        int x, y;
        worldToMap(reflector.point.x, reflector.point.y, x, y);

        if (isValidMapPoint(x, y)) {
    
            int radius = static_cast<int>(reflector.diameter / resolution_);
            // 增强周围区域
            for (int dx = -radius; dx <= radius; ++dx) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;

                   if (isValidMapPoint(nx, ny)) {
                       setCellValue(nx, ny, nav2_util::OCC_GRID_OCCUPIED);
                   }
                }
            }
        }
    }
}

} // namespace custom_map_builder