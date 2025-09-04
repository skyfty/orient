cmake_minimum_required(VERSION 3.8)
project(orient_controller)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(${rclcpp_VERSION_MAJOR} GREATER_EQUAL 20)
    add_compile_definitions(RCLCPP_SUPPORTS_MATCHED_CALLBACKS)
endif()


set(executable_name controller_server)
set(library_name ${executable_name}_core)

ament_auto_add_executable(${executable_name}
  src/main.cpp
)
ament_auto_add_library(${library_name} SHARED src/controller_server.cpp)

target_link_libraries(${executable_name} ${library_name})
rclcpp_components_register_nodes(${library_name} "orient_controller::ControllerServer")

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_auto_add_library(orient_regulated_pure_pursuit_controller SHARED src/regulated_pure_pursuit_controller.cpp src/utils.cpp)
ament_auto_add_library(orient_ackermann_controller SHARED src/ackermann_controller.cpp src/utils.cpp)
ament_auto_add_library(orient_ackermann_backup_controller SHARED src/ackermann_backup_controller.cpp src/utils.cpp)
ament_auto_add_library(goal_checker SHARED src/goal_checker.cpp )
ament_auto_add_library(position_goal_checker SHARED  src/position_goal_checker.cpp)
ament_auto_add_library(orient_rotation_shim_controller SHARED src/rotation_shim_controller.cpp src/utils.cpp)
ament_auto_add_library(orient_aline_shim_controller SHARED src/aline_shim_controller.cpp src/utils.cpp)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)


install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
ament_auto_package()