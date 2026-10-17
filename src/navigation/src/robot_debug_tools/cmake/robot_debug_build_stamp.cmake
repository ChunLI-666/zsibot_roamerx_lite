# Target-specific generated data; no shared library's hash is used as a caller's hash.
set(_ROBOT_DEBUG_STAMP_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")
function(robot_debug_build_stamp target repository_id artifact_id)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_generator "${_ROBOT_DEBUG_STAMP_CMAKE_DIR}/generate_build_stamp.py")
  if(NOT EXISTS "${_generator}")
    set(_generator "${_ROBOT_DEBUG_STAMP_CMAKE_DIR}/../scripts/generate_build_stamp.py")
  endif()
  set(_dir "${CMAKE_CURRENT_BINARY_DIR}/debug_build/${target}")
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _config)
  set(_context "${CMAKE_CXX_COMPILER}|${CMAKE_CXX_COMPILER_VERSION}|${CMAKE_BUILD_TYPE}|${CMAKE_CXX_FLAGS}|${CMAKE_CXX_FLAGS_${_config}}|${CMAKE_SHARED_LINKER_FLAGS}")
  add_custom_target(${target}_debug_stamp ALL
    COMMAND ${Python3_EXECUTABLE} "${_generator}" --repo "${CMAKE_CURRENT_SOURCE_DIR}"
      --repository-id "${repository_id}" --artifact "${artifact_id}"
      --build-context "${_context}|$<JOIN:$<TARGET_PROPERTY:${target},COMPILE_OPTIONS>,,>|$<JOIN:$<TARGET_PROPERTY:${target},COMPILE_DEFINITIONS>,,>|$<JOIN:$<TARGET_PROPERTY:${target},LINK_OPTIONS>,,>"
      --header "${_dir}/robot_debug_build.hpp" --json "${_dir}/debug_build.json"
    BYPRODUCTS "${_dir}/robot_debug_build.hpp" "${_dir}/debug_build.json"
    VERBATIM)
  add_dependencies(${target} ${target}_debug_stamp)
  target_include_directories(${target} PRIVATE "${_dir}")
  install(FILES "${_dir}/debug_build.json" DESTINATION share/${PROJECT_NAME}/debug_build/${target})
endfunction()
