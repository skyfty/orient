cmake_minimum_required(VERSION 3.5)
project(orient_utils)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(orient_interfaces REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


include_directories(
  include
)

set(library_name orient_utils)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav2_util
  orient_interfaces
)

# Main library
add_library(${library_name} SHARED
  src/transform.cpp
  src/estimate.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
