# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

include(dart_fetch_git_repo)

# Fetch GoogleTest
dart_fetch_git_repo(
  PROJECT_NAME googletest
  WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/.deps
  GIT_URL https://github.com/google/googletest.git
  GIT_TAG release-1.11.0
)

# Set options
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

# Adds the targers: gtest, gtest_main, gmock, gmock_main
add_subdirectory(
  ${googletest_SOURCE_DIR}
  ${googletest_BINARY_DIR}
  EXCLUDE_FROM_ALL
)

#===============================================================================
# This function uses following global properties:
# - DART_UNITTESTS
# - DART_${test_type}_TESTS
#
# Usage:
#   dart_add_test("unit" test_UnitTestA) # assumed source is test_UnitTestA.cpp
#   dart_add_test("unit" test_UnitTestB test_SourceB1.cpp)
#   dart_add_test("unit" test_UnitTestA test_SourceC1.cpp test_SourceC2.cpp)
#===============================================================================
function(dart_add_test test_type target_name) # ARGN for source files

  dart_property_add(DART_${test_type}_TESTS ${target_name})

  if(${ARGC} GREATER 2)
    set(sources ${ARGN})
  else()
    set(sources "${target_name}.cpp")
  endif()

  add_executable(${target_name} ${sources})
  add_test(${target_name} ${target_name})
  target_link_libraries(${target_name}
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimization
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-planning
    gtest
    gtest_main
  )

  dart_format_add(${sources})

endfunction()

#===============================================================================
# Usage:
#   dart_get_tests("integration" compreshensive_tests)
#   foreach(test ${compreshensive_tests})
#     message(STATUS "Test: ${test})
#   endforeach()
#===============================================================================
function(dart_get_tests output_var test_type)
  get_property(var GLOBAL PROPERTY DART_${test_type}_TESTS)
  set(${output_var} ${var} PARENT_SCOPE)
endfunction()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)

# We categorize tests as:
# - "integration": high level tests to verify the combination of several
#   components are correctly performs together
# - "regression": issue wise tests to verify that the GitHub issues are still
#   fixed even after further changes are made
# - "unit": low level tests for one or few classes and functions to verify that
#   they performs correctly as expected
if(NOT MSVC)
  # Disabled due to limitied disk space in CI image
  # add_subdirectory(integration)
  # add_subdirectory(regression)
endif()
add_subdirectory(unit)

dart_get_tests(integration_tests "integration")
dart_get_tests(regression_tests "regression")
dart_get_tests(unit_tests "TEST_UNIT")

# Print tests
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tests ]")
  foreach(test ${integration_tests})
    message(STATUS "Adding test: integration/${test}")
  endforeach()
  foreach(test ${regression_tests})
    message(STATUS "Adding test: regression/${test}")
  endforeach()
  foreach(test ${unit_tests})
    message(STATUS "Adding test: unit/${test}")
  endforeach()
else()
  list(LENGTH integration_tests integration_tests_len)
  list(LENGTH regression_tests regression_tests_len)
  list(LENGTH unit_tests unit_tests_len)
  math(
    EXPR tests_len
    "${integration_tests_len} + ${regression_tests_len} + ${unit_tests_len}"
  )
  message(STATUS "Adding ${tests_len} tests ("
      "integration: ${integration_tests_len}, "
      "regression: ${regression_tests_len}, "
      "unit: ${unit_tests_len}"
      ")"
  )
endif()

# Add custom target to build all the tests as a single target
add_custom_target(
  tests
  DEPENDS ${integration_tests} ${regression_tests} ${unit_tests}
)
