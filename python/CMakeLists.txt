# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Fetch pybind11
# Check latest release: https://github.com/pybind/pybind11/releases
include(dart_fetch_git_repo)
dart_fetch_git_repo(
  PROJECT_NAME pybind11
  WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/.deps
  GIT_URL https://github.com/pybind/pybind11.git
  GIT_TAG v2.7.1
)

# Need to set PYBIND11_PYTHON_VERSION before finding pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})

# Adds the targers: benchmark, benchmark_main
add_subdirectory(
  ${pybind11_SOURCE_DIR}
  ${pybind11_BINARY_DIR}
  EXCLUDE_FROM_ALL
)

if(NOT pybind11_FOUND)
  message(WARNING "Disabling <dartpy> due to failure of fetching <pybind11>.")
  return()
endif()

set(DART_DARTPY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/dartpy")

# Python binding module name
set(DARTPY_MODULE dartpy)

add_subdirectory(dartpy)  # should be included than any other subdirectories for dartpy target
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)
