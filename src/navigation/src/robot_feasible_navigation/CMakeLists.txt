cmake_minimum_required(VERSION 3.16)
project(robot_feasible_navigation)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(nlohmann_json REQUIRED)

set(common_dependencies
  rclcpp rclcpp_action rclcpp_lifecycle rclcpp_components nav2_core nav2_util
  nav2_msgs nav2_costmap_2d nav_2d_msgs nav_2d_utils geometry_msgs nav_msgs
  std_msgs visualization_msgs builtin_interfaces tf2 tf2_ros tf2_geometry_msgs pluginlib angles)

option(RFN_BUILD_SEGMENT_CONTROLLER "Build ordered FollowPath server as well as the planner" ON)
# Stateless cell proof and original-goal wire protocol, usable without the B package.
add_library(rfn_goal_cell_binding SHARED src/goal_cell_binding.cpp)
target_include_directories(rfn_goal_cell_binding PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rfn_goal_cell_binding nav2_costmap_2d nav_msgs tf2 tf2_geometry_msgs)
install(TARGETS rfn_goal_cell_binding EXPORT export_goal_cell_binding
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(RFN_BUILD_SEGMENT_CONTROLLER)
add_library(segment_controller_core SHARED
  src/segment_coordinator.cpp src/segment_controller_server.cpp)
target_include_directories(segment_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(segment_controller_core ${common_dependencies})
target_link_libraries(segment_controller_core rfn_goal_cell_binding)
rclcpp_components_register_nodes(segment_controller_core
  "robot_feasible_navigation::SegmentControllerServer")
add_executable(segment_controller_server src/segment_controller_main.cpp)
target_link_libraries(segment_controller_server segment_controller_core)
ament_target_dependencies(segment_controller_server rclcpp)
endif()

set(smac_vendor ${CMAKE_CURRENT_SOURCE_DIR}/vendor/nav2_smac_planner)
add_library(safe_lattice_planner SHARED
  src/safe_lattice_planner.cpp
  ${smac_vendor}/src/smac_planner_lattice.cpp
  ${smac_vendor}/src/a_star.cpp
  ${smac_vendor}/src/collision_checker.cpp
  ${smac_vendor}/src/smoother.cpp
  ${smac_vendor}/src/analytic_expansion.cpp
  ${smac_vendor}/src/node_hybrid.cpp
  ${smac_vendor}/src/node_lattice.cpp
  ${smac_vendor}/src/costmap_downsampler.cpp
  ${smac_vendor}/src/node_2d.cpp
  ${smac_vendor}/src/node_basic.cpp)
target_include_directories(safe_lattice_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${smac_vendor}/include ${OMPL_INCLUDE_DIRS})
ament_target_dependencies(safe_lattice_planner ${common_dependencies})
target_link_libraries(safe_lattice_planner rfn_goal_cell_binding Eigen3::Eigen nlohmann_json::nlohmann_json ${OMPL_LIBRARIES})
pluginlib_export_plugin_description_file(nav2_core plugin_lattice.xml)

install(TARGETS safe_lattice_planner
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
if(RFN_BUILD_SEGMENT_CONTROLLER)
install(TARGETS segment_controller_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS segment_controller_server RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY include/ DESTINATION include PATTERN "safe_lattice_planner.hpp" EXCLUDE)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)
install(DIRECTORY vendor/ DESTINATION share/${PROJECT_NAME}/vendor
  FILES_MATCHING PATTERN "LICENSE*" PATTERN "*manifest*.json")
install(FILES ${smac_vendor}/include/nav2_smac_planner/thirdparty/robin_hood.h
  DESTINATION share/${PROJECT_NAME}/vendor/nav2_smac_planner/thirdparty)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_footprint_sweep test/test_footprint_sweep.cpp)
  target_include_directories(test_footprint_sweep PRIVATE include)
  ament_target_dependencies(test_footprint_sweep nav2_costmap_2d)
  if(RFN_BUILD_SEGMENT_CONTROLLER)
  ament_add_gtest(test_segment_coordinator
    test/test_segment_coordinator.cpp src/segment_coordinator.cpp)
  target_include_directories(test_segment_coordinator PRIVATE include)
  ament_target_dependencies(test_segment_coordinator ${common_dependencies})
  endif()
  ament_add_gtest(test_lattice_safety test/test_lattice_safety.cpp)
  target_include_directories(test_lattice_safety PRIVATE include ${smac_vendor}/include ${OMPL_INCLUDE_DIRS})
  target_link_libraries(test_lattice_safety safe_lattice_planner Eigen3::Eigen ${OMPL_LIBRARIES})
  ament_target_dependencies(test_lattice_safety ${common_dependencies})
  set_tests_properties(test_lattice_safety PROPERTIES
    RUN_SERIAL TRUE TIMEOUT 90)
  ament_add_gtest(test_goal_cell_binding test/test_goal_cell_binding.cpp src/segment_coordinator.cpp)
  target_include_directories(test_goal_cell_binding PRIVATE include)
  target_link_libraries(test_goal_cell_binding rfn_goal_cell_binding)
  ament_target_dependencies(test_goal_cell_binding ${common_dependencies})
  ament_add_pytest_test(test_primitives test/test_primitives.py)
  set(test_environment
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=190;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;PYTHONDONTWRITEBYTECODE=1;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/${CMAKE_LIBRARY_ARCHITECTURE}:$ENV{LD_LIBRARY_PATH}")
  set_tests_properties(test_footprint_sweep test_lattice_safety test_primitives test_goal_cell_binding PROPERTIES
    ENVIRONMENT "${test_environment}")
  if(RFN_BUILD_SEGMENT_CONTROLLER)
    set_tests_properties(test_segment_coordinator PROPERTIES ENVIRONMENT "${test_environment}")
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_goal_cell_binding HAS_LIBRARY_TARGET)
ament_export_libraries(rfn_goal_cell_binding)
if(RFN_BUILD_SEGMENT_CONTROLLER)
ament_export_libraries(segment_controller_core)
endif()
ament_export_dependencies(${common_dependencies})
ament_package()
