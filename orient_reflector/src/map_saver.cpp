#include <cstdio>
#include <string>
#include <fstream>
#include <optional>
#include <boost/smart_ptr.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "orient_interfaces/msg/reflector_grid.hpp"     // CHANGE
#include "yaml-cpp/yaml.h"
#include "orient_reflector/utils.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_util/occ_grid_values.hpp"
#include "orient_interfaces/srv/get_reflector_map.hpp"     // CHANGE

#include "nav2_map_server/map_io.hpp"

using namespace std::placeholders;
using namespace orient_reflector;

const char * USAGE_STRING{
  "Usage:\n"
  "  map_saver [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "  -t <map_topic>\n"
  "  -f <mapname>\n"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};


/* Map output part */

struct SaveParameters
{
  std::string reflector_file_name{""};
};

typedef enum
{
  COMMAND_MAP_TOPIC,
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
ARGUMENTS_STATUS parse_arguments(int argc, char ** argv,
  SaveParameters & save_parameters)
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
            save_parameters.reflector_file_name = *it;
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

class MapSaver : public rclcpp::Node {
public:
    MapSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("orient_map_saver_node", options) {
            
        // Declare the node parameters
        declare_parameter("save_map_timeout", 10.0);

        save_map_timeout_ = std::make_shared<rclcpp::Duration>(
            rclcpp::Duration::from_seconds(get_parameter("save_map_timeout").as_double()));
    }

    bool saveReflector(const std::string & filename, const orient_interfaces::msg::ReflectorMap &reflector_map) {
        YAML::Node reflector_map_node = YAML::LoadFile(filename);  
        if (!reflector_map_node) {  
            return false;
        }
        std::ofstream os(filename, std::ios::trunc); 
        if (!os.is_open())   {
            return false;
        }
        double origin_xspan = reflector_map.origin.position.x - reflector_map_node["origin"][0].as<double>();
        double origin_yspan = reflector_map.origin.position.y - reflector_map_node["origin"][1].as<double>();

        YAML::Node     reflectors_nodes = YAML::Node(YAML::NodeType::Sequence);
        for (const orient_interfaces::msg::Reflector &reflector: reflector_map.reflectors) {
            YAML::Node reflector_node;
            reflector_node["id"] = reflector.id;
            reflector_node["diameter"] = reflector.diameter;
            reflector_node["intensity"] = reflector.intensity;
            reflector_node["x"] = reflector.point.x + origin_xspan;    
            reflector_node["y"] = reflector.point.y + origin_yspan;
            reflectors_nodes.push_back(reflector_node);
        }
        reflector_map_node["reflectors"] = reflectors_nodes;

        os << reflector_map_node << std::endl;
        return true;
    }

    bool saveMapTopicToFile(const SaveParameters & save_parameters)
    {
        // Local copies of map_topic and save_parameters that could be changed
        SaveParameters save_parameters_loc = save_parameters;

        RCLCPP_INFO(
            get_logger(), "Saving map from  topic to \'%s\' file",
            save_parameters_loc.reflector_file_name.c_str());
        std::string reflector_file_name = save_parameters_loc.reflector_file_name + ".yaml";

        try {
              // Create new CallbackGroup for map_sub
            auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);

            // Create SingleThreadedExecutor to spin map_sub in callback_group
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_callback_group(callback_group, get_node_base_interface());
      
            {
              std::promise<orient_interfaces::msg::ReflectorMap::SharedPtr> prom;
              std::future<orient_interfaces::msg::ReflectorMap::SharedPtr> future_result = prom.get_future();
              // A callback function that receives map message from subscribed topic
              auto mapCallback = [&prom](const orient_interfaces::msg::ReflectorMap::SharedPtr msg) -> void {
                  prom.set_value(msg);
              };
              auto option = rclcpp::SubscriptionOptions();
              option.callback_group = callback_group;
              auto map_sub = create_subscription<orient_interfaces::msg::ReflectorMap>(
                "reflector/map", 
                rclcpp::QoS (rclcpp::KeepLast(1)).transient_local().reliable(), mapCallback, option);

              auto timeout = save_map_timeout_->to_chrono<std::chrono::nanoseconds>();
              auto status = executor.spin_until_future_complete(future_result, timeout);
              if (status != rclcpp::FutureReturnCode::SUCCESS) {
                  RCLCPP_ERROR(get_logger(), "Failed to spin map subscription");
                  return false;
              }
              // map_sub is no more needed
              map_sub.reset();
              // Map message received. Saving it to file
              orient_interfaces::msg::ReflectorMap::SharedPtr map_msg = future_result.get();
              if (saveReflector(reflector_file_name, *map_msg)) {
                  RCLCPP_INFO(get_logger(), "Map saved successfully");
                  return true;
              } else {
                  RCLCPP_ERROR(get_logger(), "Failed to save the map");
                  return false;
              }
            }
   
        } catch (std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Failed to save the map: %s", e.what());
            return false;
        }

        return false;
    }

private:
  // The timeout for saving the map in service
  std::shared_ptr<rclcpp::Duration> save_map_timeout_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Parse CLI-arguments
  SaveParameters save_parameters;
  switch (parse_arguments(argc, argv, save_parameters)) {
    case ARGUMENTS_INVALID:
      rclcpp::shutdown();
      return -1;
    case HELP_MESSAGE:
      rclcpp::shutdown();
      return 0;
    case ARGUMENTS_VALID:
      break;
  }

  int retcode;
  try {
    auto map_saver = std::make_shared<MapSaver>();
    if (map_saver->saveMapTopicToFile(save_parameters)) {
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
