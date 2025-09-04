cmake_minimum_required(VERSION 3.8)
project(orient_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_library(PahoMqttC_LIBRARY libpaho-mqtt3as.so.1 REQUIRED)
find_library(PahoMqttCpp_LIBRARY libpaho-mqttpp3.so.1 REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_executable(orient_mqtt src/main.cpp)
target_link_libraries(orient_mqtt
  ${PahoMqttC_LIBRARY}
  ${PahoMqttCpp_LIBRARY}
)

ament_auto_package()
