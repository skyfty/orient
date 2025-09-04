cmake_minimum_required(VERSION 3.8)
project(orient_reflector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(PCL REQUIRED)

# find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

ament_auto_find_build_dependencies()

if(${rclcpp_VERSION_MAJOR} GREATER_EQUAL 20)
    add_compile_definitions(RCLCPP_SUPPORTS_MATCHED_CALLBACKS)
endif()

include_directories(
  include
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${CERES_INCLUDES}
  ${PCL_INCLUDE_DIRS} 
)

ament_auto_add_library(orient_reflector_core SHARED src/utils.cpp src/urdf/model.cpp src/sliding_window_filter.cpp  src/localization.cpp)
target_link_libraries(orient_reflector_core ${CERES_LIBRARIES} ${PCL_LIBRARIES}  ${urdfdom_LIBRARIES})
rclcpp_components_register_nodes(orient_reflector_core "orient_reflector::Localization")

ament_auto_add_executable(localization src/localization_main.cpp)
target_link_libraries(localization orient_reflector_core)

ament_auto_add_executable(grid_mapper src/grid_mapper.cpp src/probability_grid.cpp)
target_link_libraries(grid_mapper orient_reflector_core ${OpenCV_LIBRARIES})

ament_auto_add_executable(saver src/map_saver.cpp)

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_auto_package()