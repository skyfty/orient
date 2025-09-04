cmake_minimum_required(VERSION 3.8)
project(orient_govern)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(govern_core SHARED
  src/govern.cpp
  src/navigate_through_poses.cpp
)
rclcpp_components_register_nodes(govern_core "orient_govern::Govern")

ament_auto_add_executable(govern
  src/main.cpp
)

if(BUILD_TESTING)
  ament_auto_add_executable(test_govern_server_exec
    test/test_govern_server.cpp
    test/govern_server_tester.cpp
  )
endif()

ament_auto_package()