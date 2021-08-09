# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Python binding module name
set(DARTPY_MODULE dartpy)

# Check required modules
set(required_modules
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimization
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-gui
)
foreach(required_module ${required_modules})
  if(NOT TARGET ${required_module})
    message(WARNING "Disabled [${DARTPY_MODULE}] due to missing [${required_module}]")
    return()
  endif()
endforeach()

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

# Include pybind11
add_subdirectory(
  ${pybind11_SOURCE_DIR}
  ${pybind11_BINARY_DIR}
  EXCLUDE_FROM_ALL
)
if(NOT pybind11_FOUND)
  message(WARNING "Disabling [dartpy] due to failure of fetching [pybind11].")
  return()
endif()

# PythonInterp
find_package(PythonInterp ${DARTPY_PYTHON_VERSION} QUIET)
if(NOT PythonInterp_FOUND AND NOT PYTHONINTERP_FOUND)
  message(WARNING "Disabling [dartpy] due to missing [PythonInterp].")
  return()
endif()

# PythonLibs
find_package(PythonLibs ${DARTPY_PYTHON_VERSION} QUIET)
if(NOT PythonLibs_FOUND AND NOT PYTHONLIBS_FOUND)
  message(WARNING "Disabling [dartpy] due to missing [PythonLibs].")
  return()
endif()

set(DART_DARTPY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/dartpy")

add_subdirectory(dartpy)  # should be included than any other subdirectories for dartpy target
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)
