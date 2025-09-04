// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <string>
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <sstream>
#include <unordered_set>
#include <iomanip>

#include "govern_server_tester.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT

namespace orient_tests
{

GovernTester::GovernTester(const rclcpp::NodeOptions &options)
: Node("GovernTester",options){
  task_publisher_ = create_publisher<orient_interfaces::msg::Task>("govern/task", rclcpp::ServicesQoS());
  
}


void GovernTester::activate()
{

  // Launch a thread to process the messages for this node


  std::cout << "Starting the orient_govern tester" << std::endl;
  {

    orient_interfaces::msg::Task task;
    task.header.stamp = now();
    task.header.frame_id = "map";
    task.task_id = "task1";
    task.behavior = "follow_path.xml";

    {
      orient_interfaces::msg::CheckPoint p1;
      p1.x = 20;
      p1.y = 16;
      task.points.push_back(p1);
    }
 
    {
      orient_interfaces::msg::CheckPoint p1;
      p1.x = 39;
      p1.y = 19;
      task.points.push_back(p1);
    }
    task_publisher_->publish(task);
    std::cout << "Starting the orient_govern tester" << std::endl;


  }


}



}  // namespace nav2_system_tests
