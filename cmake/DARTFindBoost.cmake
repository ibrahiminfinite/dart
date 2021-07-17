# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(DART_MIN_BOOST_VERSION 1.65.0 CACHE INTERNAL "Boost min version requirement" FORCE)
  if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
  endif()
  add_definitions(-DBOOST_TEST_DYN_LINK)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
  if(DART_VERBOSE)
    find_package(Boost ${DART_MIN_BOOST_VERSION} REQUIRED COMPONENTS system filesystem)
  else()
    find_package(Boost ${DART_MIN_BOOST_VERSION} QUIET REQUIRED COMPONENTS system filesystem)
  endif()
  if(NOT TARGET Boost::system)
    add_library(Boost::system INTERFACE IMPORTED)
    set_target_properties(Boost::system PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${Boost_SYSTEM_LIBRARY}"
    )
  endif()
  if(NOT TARGET Boost::filesystem)
    add_library(Boost::filesystem INTERFACE IMPORTED)
    set_target_properties(Boost::filesystem PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${Boost_FILESYSTEM_LIBRARY}"
    )
  endif()
endif()
