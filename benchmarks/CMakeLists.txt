# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Fix building benchmark on macOS Catalina
if(APPLE)
  if(${CMAKE_SYSTEM_VERSION} VERSION_LESS 20)
    message(WARNING "Building benchmarks on macOS Catalina and lower is disabled")
    return()
  endif()
endif()

# Fetch GoogleTest
include(dart_fetch_git_repo)
dart_fetch_git_repo(
  PROJECT_NAME benchmark
  WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/.deps
  GIT_URL https://github.com/google/benchmark.git
  GIT_TAG v1.5.5
)

# Set options
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)

# Adds the targers: benchmark, benchmark_main
add_subdirectory(
  ${benchmark_SOURCE_DIR}
  ${benchmark_BINARY_DIR}
  EXCLUDE_FROM_ALL
)

add_subdirectory(unit)
add_subdirectory(integration)

function(dart_get_benchmarks output_var test_type)
  get_property(var GLOBAL PROPERTY DART_${test_type}_BENCHMARKS)
  set(${output_var} ${var} PARENT_SCOPE)
endfunction()

dart_get_benchmarks(integration_benchmarks "BENCHMARK_INTEGRATION")
dart_get_benchmarks(unit_benchmarks "BENCHMARK_UNIT")

# Add custom target to build all the benchmarks as a single target
add_custom_target(
  benchmarks
  DEPENDS ${integration_benchmarks} ${unit_benchmarks}
)
