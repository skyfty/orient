#include <cstdio>
#include <string>
#include <fstream>
#include <optional>
#include <boost/smart_ptr.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"
#include "orient_interfaces/srv/load_check_point.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_map.hpp"             // CHANGE
#include "orient_checkpoint/utils.hpp"

#include "orient_interfaces/srv/get_check_point_map.hpp"             // CHANGE

using namespace std::placeholders;
using namespace orient_interfaces::srv;

namespace orient_checkpoint {


const char * USAGE_STRING{
  "Usage:\n"
  "  saver [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "  -f <mapname>\n"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};


/* Map output part */

struct SaveParameters
{
  std::string nav_graph_file{""};
};

typedef enum
{
  COMMAND_REFLECTOR_FILE_NAME,
} COMMAND_TYPE;

typedef enum
{
  ARGUMENTS_INVALID,
  ARGUMENTS_VALID,
  HELP_MESSAGE
} ARGUMENTS_STATUS;

struct cmd_struct
{
  const char * cmd;
  COMMAND_TYPE command_type;
};
// Arguments parser
// Input parameters: logger, argc, argv
// Output parameters: map_topic, save_parameters
ARGUMENTS_STATUS parse_arguments(int argc, char ** argv, SaveParameters & save_parameters)
{
  const struct cmd_struct commands[] = {
    {"-f", COMMAND_REFLECTOR_FILE_NAME},
  };

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;

  size_t cmd_size = sizeof(commands) / sizeof(commands[0]);
  size_t i;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      return HELP_MESSAGE;
    }
    if (*it == "--ros-args") {
      break;
    }
    for (i = 0; i < cmd_size; i++) {
      if (commands[i].cmd == *it) {
        if ((it + 1) == arguments.end()) {
          return ARGUMENTS_INVALID;
        }
        it++;
        switch (commands[i].command_type) {
          case COMMAND_REFLECTOR_FILE_NAME:
            save_parameters.nav_graph_file = *it;
            break;
        }
        break;
      }
    }
    if (i == cmd_size) {
      return ARGUMENTS_INVALID;
    }
  }

  return ARGUMENTS_VALID;
}


class Saver : public rclcpp::Node {
public:
    Saver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("orient_checkpoint_saver_node", options) {
            
        // Declare the node parameters
        declare_parameter("save_timeout", 10.0);
        declare_parameter("save_map_timeout", 10.0);

        save_map_timeout_ = std::make_shared<rclcpp::Duration>(
            rclcpp::Duration::from_seconds(get_parameter("save_map_timeout").as_double()));
    }

    bool saveToFile(const SaveParameters & save_parameters)
    {
        // Local copies of map_topic and save_parameters that could be changed
        SaveParameters save_parameters_loc = save_parameters;
        if (save_parameters_loc.nav_graph_file.empty()) {
          save_parameters_loc.nav_graph_file = "0.yaml";
        }
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;


        auto callback_group_ = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive,
          false);
        callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());

        rclcpp::Client<orient_interfaces::srv::GetCheckPointMap>::SharedPtr client;

        client = create_client<orient_interfaces::srv::GetCheckPointMap>(
          "get_checkpoint_map",
          rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);


        // 1.等待服务端上线
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            //等待时检测rclcpp的状态
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }

        // 2.创建请求
        auto request = std::make_shared<GetCheckPointMap::Request>();
        auto future_handle = client->async_send_request(request);
        rclcpp::FutureReturnCode rc = callback_group_executor_.spin_until_future_complete(future_handle, std::chrono::seconds(10));
        if (rc != rclcpp::FutureReturnCode::SUCCESS) {
            return false;
        }
        auto result = future_handle.get();
        orient_checkpoint::save_yaml(save_parameters_loc.nav_graph_file, result->map);
        RCLCPP_INFO(get_logger(), "保存地图成功");
        return false;
    }

private:
  // The timeout for saving the map in service
  std::shared_ptr<rclcpp::Duration> save_map_timeout_;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Parse CLI-arguments
  orient_checkpoint::SaveParameters save_parameters;
  switch (parse_arguments(argc, argv, save_parameters)) {
    case orient_checkpoint::HELP_MESSAGE:
      rclcpp::shutdown();
      return 0;
  }

  int retcode;
  try {
    auto map_saver = std::make_shared<orient_checkpoint::Saver>();
    if (map_saver->saveToFile(save_parameters)) {
      retcode = 0;
    } else {
      retcode = 1;
    }
  } catch (std::exception & e) {
    retcode = -1;
  }

  // Exit
  rclcpp::shutdown();
  return retcode;
}
