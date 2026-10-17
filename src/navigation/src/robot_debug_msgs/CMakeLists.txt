cmake_minimum_required(VERSION 3.16)
project(robot_debug_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BuildStamp.msg" "msg/BuildInfo.msg" "msg/CheckerSample.msg"
  "msg/BranchSample.msg" "msg/GateSample.msg" "msg/TransportSample.msg"
  "msg/ExecutionSample.msg" "msg/StatusObservation.msg" "msg/ActionState.msg"
  "msg/DebugView.msg"
  DEPENDENCIES std_msgs builtin_interfaces unique_identifier_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
