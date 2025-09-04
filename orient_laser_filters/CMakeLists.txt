cmake_minimum_required(VERSION 3.5)
project(orient_laser_filters)

find_package(ament_cmake_auto REQUIRED)
find_package(filters REQUIRED)

ament_auto_find_build_dependencies()


if(${rclcpp_VERSION_MAJOR} GREATER_EQUAL 20)
    add_compile_definitions(RCLCPP_SUPPORTS_MATCHED_CALLBACKS)
endif()

ament_auto_add_library(laser_scan_filters SHARED src/laser_scan_filters.cpp)
ament_auto_add_library(laser_filter_chains SHARED
                         src/scan_to_scan_filter_chain.cpp)

rclcpp_components_register_node(
    laser_filter_chains
    PLUGIN "ScanToScanFilterChain"
    EXECUTABLE "scan_to_scan_filter_chain"
)

pluginlib_export_plugin_description_file(filters laser_filters_plugins.xml)
ament_auto_package(INSTALL_TO_SHARE)