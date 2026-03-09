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

# Determine architecture for zsibot SDK
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ZSIBOT_SDK_ARCH "aarch64")
  set(ZSIBOT_SDK_LIB_NAME "mc_sdk_zsl_1_aarch64")
else()
  set(ZSIBOT_SDK_ARCH "x86_64")
  set(ZSIBOT_SDK_LIB_NAME "mc_sdk_zsl_1_x86_64")
endif()

# Set zsibot SDK paths
set(ZSIBOT_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../zsibot_sdk")
set(ZSIBOT_SDK_INCLUDE_DIR "${ZSIBOT_SDK_DIR}/include")
set(ZSIBOT_SDK_LIB_DIR "${ZSIBOT_SDK_DIR}/lib/zsl-1/${ZSIBOT_SDK_ARCH}")

# Check if SDK exists
if(NOT EXISTS "${ZSIBOT_SDK_INCLUDE_DIR}/zsl-1/highlevel.h")
  message(FATAL_ERROR "ZsiBot SDK not found at ${ZSIBOT_SDK_INCLUDE_DIR}")
endif()

# Get absolute path for SDK library directory (for RPATH)
get_filename_component(ZSIBOT_SDK_LIB_DIR_ABS "${ZSIBOT_SDK_LIB_DIR}" ABSOLUTE)

# Link directories for SDK
link_directories(${ZSIBOT_SDK_LIB_DIR_ABS})

include_directories(
  include
  ${ZSIBOT_SDK_INCLUDE_DIR}
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
  ${ZSIBOT_SDK_LIB_DIR}/lib${ZSIBOT_SDK_LIB_NAME}.so
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

# Find SDK library file
set(SDK_LIB_FILE "${ZSIBOT_SDK_LIB_DIR_ABS}/lib${ZSIBOT_SDK_LIB_NAME}.so")
message(STATUS "ZsiBot SDK library: ${SDK_LIB_FILE}")

if(NOT EXISTS "${SDK_LIB_FILE}")
  message(FATAL_ERROR "ZsiBot SDK library not found: ${SDK_LIB_FILE}")
endif()

# Install SDK library to lib directory so it can be found at runtime
install(
  FILES "${SDK_LIB_FILE}"
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
