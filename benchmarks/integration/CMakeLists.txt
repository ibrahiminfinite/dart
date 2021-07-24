# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

set(benchmark_type "BENCHMARK_INTEGRATION")

# Glob all the benchmark files
file(GLOB_RECURSE benchmark_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "benchmark_*.cpp")
if(benchmark_files)
  list(SORT benchmark_files)
endif()

set(link_libraries)
set(link_dart_libraries
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
)

# Build the uint benchmarks
dart_build_benchmarks(
  TYPE ${benchmark_type}
  TEST_LIST integration_benchmarks
  SOURCES ${benchmark_files}
  LINK_LIBRARIES ${link_libraries}
  LINK_DART_LIBRARIES ${link_dart_libraries}
)
