# Shared by every g1_ package. Include once, near the top:
#
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)
#
#   g1_target_defaults(<target>)   include dir, C++20, warnings
#   g1_add_lint_tests()            inside if(BUILD_TESTING); clang-format, clang-tidy, ruff
#
# A missing tool warns and skips rather than failing configure.

set(G1_WORKSPACE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

# Only when the caller named none; an explicit choice always wins.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(G1_PYTHON_DIRS test launch scripts)

function(g1_target_defaults target)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(${target} PUBLIC cxx_std_20)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra)
  endif()
endfunction()

function(_g1_add_clang_format_test)
  file(GLOB_RECURSE _sources
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
  )
  if(NOT _sources)
    return()
  endif()

  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_config "${G1_WORKSPACE_ROOT}/.clang-format")
  if(NOT CLANG_FORMAT_EXECUTABLE OR NOT EXISTS "${_config}")
    message(WARNING "clang-format or ${_config} not found; skipping clang_format_check_${PROJECT_NAME}")
    return()
  endif()

  add_test(
    NAME clang_format_check_${PROJECT_NAME}
    COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_config} --dry-run --Werror ${_sources}
  )
endfunction()

function(_g1_add_clang_tidy_test)
  file(GLOB_RECURSE _sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
  if(NOT _sources)
    return()
  endif()

  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy run-clang-tidy-18 run-clang-tidy-14)
  set(_config "${G1_WORKSPACE_ROOT}/.clang-tidy")
  if(NOT RUN_CLANG_TIDY_EXECUTABLE OR NOT EXISTS "${_config}" OR NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    message(WARNING "run-clang-tidy, ${_config} or compile commands missing; skipping clang_tidy_check_${PROJECT_NAME}")
    return()
  endif()

  # Labelled so `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  add_test(
    NAME clang_tidy_check_${PROJECT_NAME}
    COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
  )
  set_tests_properties(clang_tidy_check_${PROJECT_NAME} PROPERTIES LABELS clang_tidy TIMEOUT 900)
endfunction()

function(_g1_add_ruff_test)
  set(_targets "")
  foreach(_dir IN LISTS G1_PYTHON_DIRS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
      list(APPEND _targets "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    endif()
  endforeach()

  # ruff skips extensionless files, and an installed script has no .py, so name the Python
  # ones. By shebang, because scripts/ also holds shell.
  file(GLOB _scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
  foreach(_script IN LISTS _scripts)
    get_filename_component(_name "${_script}" NAME)
    if(_name MATCHES "\\." OR IS_DIRECTORY "${_script}")
      continue()
    endif()
    file(READ "${_script}" _shebang LIMIT 32)
    if(_shebang MATCHES "^#![^\n]*python")
      list(APPEND _targets "${_script}")
    endif()
  endforeach()

  if(NOT _targets)
    return()
  endif()

  find_program(RUFF_EXECUTABLE ruff)
  set(_config "${G1_WORKSPACE_ROOT}/ruff.toml")
  if(NOT RUFF_EXECUTABLE OR NOT EXISTS "${_config}")
    message(WARNING "ruff or ${_config} not found; skipping ruff_check_${PROJECT_NAME}")
    return()
  endif()

  add_test(
    NAME ruff_check_${PROJECT_NAME}
    COMMAND ${RUFF_EXECUTABLE} check --config ${_config} ${_targets}
  )
  # From the build dir, ruff.toml's relative src= would not resolve and isort finds nothing.
  set_tests_properties(ruff_check_${PROJECT_NAME} PROPERTIES
    WORKING_DIRECTORY "${G1_WORKSPACE_ROOT}")
endfunction()

function(g1_add_lint_tests)
  _g1_add_clang_format_test()
  _g1_add_clang_tidy_test()
  _g1_add_ruff_test()
endfunction()
