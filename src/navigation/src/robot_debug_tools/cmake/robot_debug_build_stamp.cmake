# Target-specific generated data; no shared library's hash is used as a caller's hash.
set(_ROBOT_DEBUG_STAMP_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")
function(_robot_debug_source_manifest_args repository_id output_args output_manifest)
  string(MAKE_C_IDENTIFIER "${repository_id}" _repository_key)
  string(TOUPPER "${_repository_key}" _repository_key)
  set(_key "ROBOT_DEBUG_SOURCE_MANIFEST_${_repository_key}")
  set(${_key} "${${_key}}" CACHE FILEPATH "Verified complete Git archive source proof for ${repository_id}")
  set(_args "")
  set(_manifest "")
  if(NOT "${${_key}}" STREQUAL "")
    get_filename_component(_manifest "${${_key}}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT EXISTS "${_manifest}")
      message(FATAL_ERROR "${_key} does not exist: ${_manifest}")
    endif()
    set(_args --source-manifest "${_manifest}")
  endif()
  set(${output_args} "${_args}" PARENT_SCOPE)
  set(${output_manifest} "${_manifest}" PARENT_SCOPE)
endfunction()

function(robot_debug_build_stamp target repository_id artifact_id)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_generator "${_ROBOT_DEBUG_STAMP_CMAKE_DIR}/generate_build_stamp.py")
  if(NOT EXISTS "${_generator}")
    set(_generator "${_ROBOT_DEBUG_STAMP_CMAKE_DIR}/../scripts/generate_build_stamp.py")
  endif()
  set(_dir "${CMAKE_CURRENT_BINARY_DIR}/debug_build/${target}")
  _robot_debug_source_manifest_args("${repository_id}" _manifest_args _manifest)
  set(_proof_output "")
  if(_manifest)
    set(_proof_output "${_dir}/source_manifest.json")
  endif()
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _config)
  set(_context "${CMAKE_CXX_COMPILER}|${CMAKE_CXX_COMPILER_VERSION}|${CMAKE_BUILD_TYPE}|${CMAKE_CXX_FLAGS}|${CMAKE_CXX_FLAGS_${_config}}|${CMAKE_SHARED_LINKER_FLAGS}")
  add_custom_target(${target}_debug_stamp ALL
    COMMAND ${Python3_EXECUTABLE} "${_generator}" --repo "${CMAKE_CURRENT_SOURCE_DIR}"
      --repository-id "${repository_id}" --artifact "${artifact_id}"
      ${_manifest_args}
      --build-context "${_context}|$<JOIN:$<TARGET_PROPERTY:${target},COMPILE_OPTIONS>,,>|$<JOIN:$<TARGET_PROPERTY:${target},COMPILE_DEFINITIONS>,,>|$<JOIN:$<TARGET_PROPERTY:${target},LINK_OPTIONS>,,>"
      --header "${_dir}/robot_debug_build.hpp" --json "${_dir}/debug_build.json"
    BYPRODUCTS "${_dir}/robot_debug_build.hpp" "${_dir}/debug_build.json" ${_proof_output}
    VERBATIM)
  add_dependencies(${target} ${target}_debug_stamp)
  target_include_directories(${target} PRIVATE "${_dir}")
  install(FILES "${_dir}/debug_build.json" DESTINATION share/${PROJECT_NAME}/debug_build/${target})
  if(_manifest)
    install(FILES "${_proof_output}" DESTINATION share/${PROJECT_NAME}/debug_build/${target})
  endif()
endfunction()
