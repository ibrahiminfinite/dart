# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(NOT DARTPY_PYTHON_VERSION)
  set(DARTPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

# Find pybind11
#
# Need to set PYBIND11_PYTHON_VERSION before finding pybind11
#
# No need to explicitly call find_package(PythonLibs) nor find_package(PythonInterp) since the package is found when
# calling find_package(pybind11)
# See: https://pybind11.readthedocs.io/en/stable/faq.html#inconsistent-detection-of-python-version-in-cmake-and-pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})
find_package(pybind11 2.4.3 QUIET)
if(NOT pybind11_FOUND)
  message(WARNING "Disabling dartpy due to missing pybind11 >= 2.4.3.")
  return()
endif()
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE PYTHON_SITE_PACKAGES)
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()

file(GLOB_RECURSE dartpy_headers "*.h" "*.hpp")
file(GLOB_RECURSE dartpy_sources "*.cpp")

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
pybind11_add_module(${DARTPY_MODULE}
  MODULE
  ${dartpy_headers}
  ${dartpy_sources}
  EXCLUDE_FROM_ALL
)

target_include_directories(${DARTPY_MODULE}
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${DARTPY_MODULE}
  PUBLIC
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimization
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-gui
    ${PYTHON_LIBRARIES}
)

# Remove debug postfix for dartpy
set_target_properties(${DARTPY_MODULE} PROPERTIES DEBUG_POSTFIX "")

# Get the path to the bind module
set(dartpy_module_path $<TARGET_FILE:${DARTPY_MODULE}>)

# Custom target to install (copy) the bind module. This target may require
# `sudo` if the destination is a system directory.
set(install_comment "Installing ${DARTPY_MODULE}...")
if(BUILD_SHARED_LIBS)
  string(CONCAT install_comment
    "${install_comment}\n"
    "NOTE: ${DARTPY_MODULE} is built against the DART's shared libraries. "
    "Install the shared libraries to be able to import ${DARTPY_MODULE}."
  )
endif()
add_custom_target(install-${DARTPY_MODULE}
  COMMENT "${install_comment}"
  COMMAND ${CMAKE_COMMAND} -E copy ${dartpy_module_path} ${PYTHON_SITE_PACKAGES}
  DEPENDS ${install_dartpy_deps}
)

list(REMOVE_ITEM dartpy_headers
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.h
  ${CMAKE_CURRENT_LIST_DIR}/eigen_pybind.h
)
list(REMOVE_ITEM dartpy_sources
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.cpp
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
