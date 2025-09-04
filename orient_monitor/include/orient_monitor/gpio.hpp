#ifndef NAV2_COLLISION_MONITOR__GPIO_HPP_
#define NAV2_COLLISION_MONITOR__GPIO_HPP_
// Licensed under the Apache License, Version 2.0 (the "License");

#include <initializer_list>
#include <vector>
#include <map>
#include <fstream>
#include "nav2_util/robot_utils.hpp"

#include "orient_monitor/source.hpp"

namespace orient_monitor
{
class Gpio {
public:
    Gpio(const nav2_util::LifecycleNode::WeakPtr & node);
    ~Gpio();
    bool check(std::map<int, int> & pin_values);
    bool check_any();

private:
    std::map<int, std::ifstream> pins_;
};
}
#endif