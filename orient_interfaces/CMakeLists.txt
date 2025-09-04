cmake_minimum_required(VERSION 3.8)
project(orient_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Reflector.msg"
  "msg/ReflectorGrid.msg"
  "msg/ReflectorMap.msg"
  "msg/CheckPoint.msg"
  "msg/CheckPoints.msg"
  "msg/CheckPointPath.msg"
  "msg/CheckPointMap.msg"
  "msg/CheckPointState.msg"
  "msg/CheckPointStateStamp.msg"
  "msg/CheckPointStateMap.msg"
  "msg/Audio.msg"
  "msg/CollisionAction.msg"
  "msg/CheckpointTrack.msg"
  "msg/AckermannBackupPath.msg"
  "msg/Task.msg"
  "msg/TaskReport.msg"
  "msg/AgentState.msg"
  "action/FollowCheckpoints.action"
  "action/Forklift.action"
  "srv/SetTask.srv"
  "srv/SetCheckpointState.srv"
  "srv/GetCheckpointScout.srv"
  "srv/IsCheckpointEmployed.srv"
  "srv/IsCheckpointFree.srv"
  "srv/IsCheckpointTrackEmpty.srv"
  "srv/CheckPointPose.srv"
  "srv/LoadCheckPoint.srv"
  "srv/LoadReflectorMap.srv"
  "srv/GetReflectorMap.srv"
  "srv/GetCheckPointMap.srv"
  "srv/GetInitialPose.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs rmf_fleet_msgs
)
ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
