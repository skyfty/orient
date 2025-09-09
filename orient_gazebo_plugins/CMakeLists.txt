cmake_minimum_required(VERSION 3.8)
project(orient_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo fortress")
find_package(ignition-plugin1 REQUIRED)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)

include_directories(include)

#########

add_library(orient_gazebo_plugins-system SHARED
  src/gz_ros2_control_plugin.cpp
)

target_link_libraries(orient_gazebo_plugins-system
  ${GZ_SIM}
  ${GZ_PLUGIN}
)
ament_target_dependencies(orient_gazebo_plugins-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)

#########

add_library(orient_gazebo_plugins_hardware_plugins SHARED
  src/gz_system.cpp
)
ament_target_dependencies(orient_gazebo_plugins_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
)
target_link_libraries(orient_gazebo_plugins_hardware_plugins
  ${GZ_SIM}
)

## Install
install(TARGETS
  orient_gazebo_plugins_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)


ament_export_include_directories(include)
ament_export_libraries(orient_gazebo_plugins_hardware_plugins-system gz_hardware_plugins)

# Install directories
install(TARGETS orient_gazebo_plugins-system
  DESTINATION lib
)
pluginlib_export_plugin_description_file(orient_gazebo_plugins gz_hardware_plugins.xml)

# Setup the project
ament_package()
