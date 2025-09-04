cmake_minimum_required(VERSION 3.8)
project(orient_checkpoint)

# find dependencies
find_package(Boost REQUIRED COMPONENTS graph)  # Add this line to find Boost
find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
pkg_check_modules(GMAG REQUIRED GraphicsMagick++)

find_package(orient_utils REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  ${Boost_INCLUDE_DIRS}  # Add this line to include Boost headers
  ${GMAG_INCLUDE_DIRS}
)

ament_auto_add_library(checkpoint_core SHARED
  src/checkpoint.cpp
  src/utils.cpp
)
target_link_libraries(checkpoint_core ${Boost_LIBRARIES} ${GMAG_LIBRARIES} )  # Add this line to link Boost libraries
rclcpp_components_register_nodes(checkpoint_core "orient_checkpoint::CheckpointServer")

ament_auto_add_executable(checkpoint
  src/main.cpp
)

ament_auto_add_executable(saver
  src/saver.cpp
)

ament_auto_package()