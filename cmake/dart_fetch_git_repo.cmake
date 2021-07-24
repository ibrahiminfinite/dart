# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Replace this with FetchContent once CMake minimum increased to 3.11

macro(dart_fetch_git_repo)
  set(prefix _)
  set(options
  )
  set(oneValueArgs
    PROJECT_NAME
    WORKING_DIR
    GIT_URL
    GIT_TAG
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  set(${__PROJECT_NAME}_SOURCE_DIR ${__WORKING_DIR}/${__PROJECT_NAME}-src)
  set(${__PROJECT_NAME}_BINARY_DIR ${__WORKING_DIR}/${__PROJECT_NAME}-build)

  # Variables used configuring dart_fetch_git_repo_sub.cmake.in
  set(FETCH_PROJECT_NAME ${__PROJECT_NAME})
  set(FETCH_SOURCE_DIR ${${__PROJECT_NAME}_SOURCE_DIR})
  set(FETCH_BINARY_DIR ${${__PROJECT_NAME}_BINARY_DIR})
  set(FETCH_GIT_REPOSITORY ${__GIT_URL})
  set(FETCH_GIT_TAG ${__GIT_TAG})

  configure_file(
    ${DART_SOURCE_DIR}/cmake/dart_fetch_at_configure_step.cmake.in
    ${__WORKING_DIR}/${__PROJECT_NAME}/CMakeLists.txt
    @ONLY
  )

  # Unset them again
  unset(FETCH_PROJECT_NAME)
  unset(FETCH_SOURCE_DIR)
  unset(FETCH_BINARY_DIR)
  unset(FETCH_GIT_REPOSITORY)
  unset(FETCH_GIT_TAG)

  # Configure sub-project
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY ${__WORKING_DIR}/${__PROJECT_NAME}
  )

  # Build sub-project which triggers ExternalProject_Add
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY ${__WORKING_DIR}/${__PROJECT_NAME}
  )
endmacro()
