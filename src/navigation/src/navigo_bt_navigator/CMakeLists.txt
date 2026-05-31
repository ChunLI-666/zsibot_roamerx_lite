cmake_minimum_required(VERSION 3.5)
project(navigo_bt_navigator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigo_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(navigo_util REQUIRED)
find_package(navigo_core REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

# Allow deprecated API warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error)
endif()

# Suppress false positive null-dereference warning from GCC 13+ with std::string/streambuf
add_compile_options(-Wno-null-dereference)

include_directories(
  include
)

set(executable_name bt_navigator)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  navigo_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_srvs
  navigo_util
  navigo_core
  tf2_ros
)

add_library(${library_name} SHARED
  src/bt_navigator.cpp
  src/navigators/navigate_to_pose.cpp
  src/navigators/navigate_through_poses.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

# Link system ncurses explicitly to avoid conda ncurses conflict
find_library(NCURSES_LIBRARY NAMES ncurses PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} /usr/lib NO_DEFAULT_PATH)
find_library(NCURSESW_LIBRARY NAMES ncursesw PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} /usr/lib NO_DEFAULT_PATH)

target_link_libraries(${executable_name} ${library_name}
  ${NCURSES_LIBRARY}
  ${NCURSESW_LIBRARY}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "navigo_bt_navigator::BtNavigator")

install(TARGETS ${library_name}
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

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
