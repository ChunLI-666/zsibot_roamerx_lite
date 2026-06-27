cmake_minimum_required(VERSION 3.5)
project(cmd_vel_to_zsibot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

include_directories(
  include
)

set(executable_name cmd_vel_to_zsibot_node)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

# Main library
add_library(${library_name} SHARED
  src/cmd_vel_to_zsibot.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name}
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name})





# Register component
rclcpp_components_register_nodes(${library_name} "cmd_vel_to_zsibot::CmdVelToZsibot")

# Install targets
install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  ament_add_gtest(test_cmd_vel_to_zsibot
    test/test_cmd_vel_to_zsibot.cpp
  )
  ament_target_dependencies(test_cmd_vel_to_zsibot
    ${dependencies}
  )
  target_link_libraries(test_cmd_vel_to_zsibot ${library_name})
endif()

ament_package()
