# Copyright (c) 2011-2021, The DART development contributors

#===============================================================================
# Required dependencies
#===============================================================================

# fmt
find_package(fmt REQUIRED)

# Eigen3
find_package(Eigen3 3.3.4 REQUIRED CONFIG)
if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
  if(DART_DEBUG)
    message("Defining Eigen3::Eigen target.")
  endif()
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(
    Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                             "${EIGEN3_INCLUDE_DIRS}"
  )
endif()

#===============================================================================
# Optional dependencies
#===============================================================================

# TBB
find_package(TBB QUIET)

# OpenMP
find_package(OpenMP QUIET)

# spdlog
find_package(spdlog 1.3.0 QUIET)

# ccd
find_package(ccd MODULE QUIET)

# fcl
find_package(fcl MODULE QUIET)

# OpenMP
find_package(OpenGL QUIET MODULE)

# imgui
# find_package(imgui MODULE REQUIRED)

# raylib
set(raylib_USE_STATIC_LIBS OFF)
find_package(raylib QUIET CONFIG)
