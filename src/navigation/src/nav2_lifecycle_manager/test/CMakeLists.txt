ament_add_gtest_executable(test_lifecycle_gtest
  test_lifecycle_manager.cpp
)

target_link_libraries(test_lifecycle_gtest
  ${library_name}
)

ament_target_dependencies(test_lifecycle_gtest
  ${dependencies}
)

ament_add_test(test_lifecycle
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/launch_lifecycle_test.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 20
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
)

ament_add_gtest_executable(test_bond_gtest
  test_bond.cpp
)

target_link_libraries(test_bond_gtest
  ${library_name}
)

ament_target_dependencies(test_bond_gtest
  ${dependencies}
)

ament_add_test(test_bond
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/launch_bond_test.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 20
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_bond_gtest>
)

# Local lifetime regression: no navigation nodes, algorithms or actuator bridges.
ament_add_gtest(test_callback_lifetime test_callback_lifetime.cpp)
target_link_libraries(test_callback_lifetime ${library_name})
ament_target_dependencies(test_callback_lifetime ${dependencies})

find_package(class_loader REQUIRED)
add_executable(component_callback_lifetime component_callback_lifetime.cpp)
ament_target_dependencies(component_callback_lifetime
  rclcpp rclcpp_components class_loader lifecycle_msgs std_srvs)
add_test(NAME component_callback_lifetime
  COMMAND component_callback_lifetime $<TARGET_FILE:${library_name}>)
set_tests_properties(component_callback_lifetime PROPERTIES TIMEOUT 20)
add_test(NAME component_populated_client_lifetime
  COMMAND component_callback_lifetime $<TARGET_FILE:${library_name}> populated)
set_tests_properties(component_populated_client_lifetime PROPERTIES TIMEOUT 20)

# Real manager worker + intentionally unanswered lifecycle service. Each case
# runs in its own process so the old shutdown deadlock has a strict deadline.
add_executable(blocked_request_shutdown blocked_request_shutdown.cpp)
target_link_libraries(blocked_request_shutdown ${library_name})
ament_target_dependencies(blocked_request_shutdown ${dependencies})
foreach(mode ownership startup shutdown bond_active bond_owner_release bond_recovery_shutdown)
  add_test(NAME blocked_request_${mode} COMMAND blocked_request_shutdown ${mode})
  set_tests_properties(blocked_request_${mode} PROPERTIES TIMEOUT 15)
endforeach()
