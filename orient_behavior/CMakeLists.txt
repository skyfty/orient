cmake_minimum_required(VERSION 3.8)
project(orient_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(orient_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  orient_interfaces
  nav2_util
  pluginlib
  nav2_costmap_2d
)


add_library(orient_is_checkpoint_employed_condition_node SHARED src/is_checkpoint_employed.cpp)
list(APPEND plugin_libs orient_is_checkpoint_employed_condition_node)

add_library(orient_is_checkpoint_free_condition_node SHARED src/is_checkpoint_free.cpp)
list(APPEND plugin_libs orient_is_checkpoint_free_condition_node)

add_library(orient_set_checkpoint_state_action_node SHARED src/set_checkpoint_state.cpp)
list(APPEND plugin_libs orient_set_checkpoint_state_action_node)

add_library(orient_is_checkpoint_track_empty_condition_node SHARED src/is_checkpoint_track_empty.cpp)
list(APPEND plugin_libs orient_is_checkpoint_track_empty_condition_node)

add_library(orient_get_checkpoint_track_condition_node SHARED src/get_checkpoint_track.cpp)
list(APPEND plugin_libs orient_get_checkpoint_track_condition_node)

add_library(orient_distance_checkpoint_condition_node SHARED src/distance_checkpoint_condition.cpp)
list(APPEND plugin_libs orient_distance_checkpoint_condition_node)

add_library(orient_follow_path_action_node SHARED src/follow_path_action.cpp)
list(APPEND plugin_libs orient_follow_path_action_node)

add_library(orient_truncate_path_action_node SHARED src/truncate_path_action.cpp)
list(APPEND plugin_libs orient_truncate_path_action_node)

add_library(orient_get_robot_pose_node SHARED src/get_robot_pose.cpp)
list(APPEND plugin_libs orient_get_robot_pose_node)

add_library(orient_forklift_action_node SHARED src/forklift.cpp)
list(APPEND plugin_libs orient_forklift_action_node)

add_library(orient_forklift_behavior SHARED src/forklift_behavior.cpp)
list(APPEND plugin_libs orient_forklift_behavior)

add_library(orient_slowdown_controller_node SHARED src/slowdown_controller.cpp)
list(APPEND plugin_libs orient_slowdown_controller_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

ament_export_libraries(
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})

ament_package()
