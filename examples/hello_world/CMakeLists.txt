# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components io gui)
set(required_libraries dart7-io dart7-gui)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DARTSIM7 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)

message("[DEBUG] DART_FOUND: ${DART_FOUND}")
message("[DEBUG] DART7_FOUND: ${DART7_FOUND}")
message("[DEBUG] DARTSIM_FOUND: ${DARTSIM_FOUND}")
message("[DEBUG] DARTSIM7_FOUND: ${DARTSIM7_FOUND}")

message("[DEBUG] DART_LIBRARIES: ${DART_LIBRARIES}")
message("[DEBUG] DART7_LIBRARIES: ${DART7_LIBRARIES}")
message("[DEBUG] DARTSIM_LIBRARIES: ${DARTSIM_LIBRARIES}")
message("[DEBUG] DARTSIM7_LIBRARIES: ${DARTSIM7_LIBRARIES}")

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
