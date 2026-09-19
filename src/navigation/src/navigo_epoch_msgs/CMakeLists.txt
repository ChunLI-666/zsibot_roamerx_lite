cmake_minimum_required(VERSION 3.8)
project(navigo_epoch_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationIdentity.msg"
  "msg/LocalizationEpoch.msg"
  "msg/NavigationToken.msg"
  "msg/EpochPath.msg"
  "msg/NavigationIntent.msg"
  "msg/NavExecutionState.msg"
  "msg/EpochCommand.msg"
  "action/FollowPathEpoch.action"
  "action/BackUpEpoch.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
