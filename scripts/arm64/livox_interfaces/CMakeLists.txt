cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2 VERSION 1.0.0 LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
set(LIVOX_INTERFACE_SOURCE "" CACHE PATH "Paired LocMap thirdparty/livox_ros_driver directory")
foreach(_name CustomPoint CustomMsg)
  if(NOT EXISTS "${LIVOX_INTERFACE_SOURCE}/msg/${_name}.msg")
    message(FATAL_ERROR "Missing canonical Livox interface: ${LIVOX_INTERFACE_SOURCE}/msg/${_name}.msg")
  endif()
endforeach()
rosidl_generate_interfaces(${PROJECT_NAME}
  "${LIVOX_INTERFACE_SOURCE}:msg/CustomPoint.msg"
  "${LIVOX_INTERFACE_SOURCE}:msg/CustomMsg.msg"
  DEPENDENCIES std_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
