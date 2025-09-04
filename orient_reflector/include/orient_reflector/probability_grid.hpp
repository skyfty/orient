#ifndef PROBABILITY_GRID_HPP
#define PROBABILITY_GRID_HPP

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <mutex>
#include "orient_interfaces/msg/reflector.hpp"     // CHANGE

namespace orient_reflector {

class ProbabilityGrid {
public:
    ProbabilityGrid(double resolution, int width, int height, 
                    double origin_x, double origin_y);
    
    // 坐标转换
    void worldToMap(double wx, double wy, int& mx, int& my) const;
    void mapToWorld(int mx, int my, double& wx, double& wy) const;
    bool isValidMapPoint(int mx, int my) const;
    
    // 地图操作
    void updateCell(int x, int y, bool hit);
    void setCellValue(int x, int y, int8_t value);
    int8_t getCellValue(int x, int y) const;
    
    // 地图更新
    void updateFromScan(const Eigen::Vector2d& robot_position, 
                        const std::vector<Eigen::Vector2d>& scan_points);
    
    void expandIfNeeded(double robot_x, double robot_y);
    
    // 反光柱强化
    void reinforceReflectors(const std::vector<orient_interfaces::msg::Reflector>& reflectors);
    
    // 获取地图数据
    const std::vector<int8_t>& getGridData() const { return grid_data_; }
    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    
    // Bresenham直线算法
    std::vector<Eigen::Vector2i> bresenhamLine(int x0, int y0, int x1, int y1) const;
    
    // 扩展地图实现
    void expandGrid(int new_width, int new_height, 
                    double new_origin_x, double new_origin_y);
    
private:
    // 地图数据
    std::vector<int8_t> grid_data_;
    
    // 地图参数
    double resolution_;  // 米/像素
    int width_;          // 像素宽度
    int height_;         // 像素高度
    double origin_x_;    // 地图原点X (世界坐标)
    double origin_y_;    // 地图原点Y (世界坐标)
    
    // 概率参数
    double hit_prob_ = 0.7;     // 命中概率增益
    double miss_prob_ = 0.4;    // 未命中概率增益
    double min_prob_ = 0.1;     // 最小概率
    double max_prob_ = 0.9;     // 最大概率
    
    // 线程安全
    mutable std::mutex grid_mutex_;
};

} // namespace custom_map_builder

#endif // PROBABILITY_GRID_HPP