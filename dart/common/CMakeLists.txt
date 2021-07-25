# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name common)
set(component_dependency_packages )
set(link_libraries_public Eigen3::Eigen)
if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  list(APPEND component_dependency_packages Boost)
  list(APPEND link_libraries_public Boost::boost Boost::system Boost::filesystem)
endif()

if(TARGET fmt::fmt-header-only)
  list(APPEND link_libraries_public fmt::fmt-header-only)
elseif(TARGET fmt::fmt)
  list(APPEND link_libraries_public fmt::fmt)
else()
  message(FATAL_ERROR "Failed to find fmt targets")
endif()

if(spdlog_FOUND)
  if(TARGET spdlog::spdlog_header_only)
    list(APPEND link_libraries_public spdlog::spdlog_header_only)
  elseif(TARGET spdlog::spdlog)
    list(APPEND link_libraries_public spdlog::spdlog)
  else()
    message(FATAL_ERROR "Failed to find spdlog targets")
  endif()
endif()

# Build DART with all available SIMD instructions
if(DART_ENABLE_SIMD)
  if(MSVC)
    list(APPEND compile_options_public /arch:SSE2 /arch:AVX /arch:AVX2)
    # /arch:SSE2 option is effective only for x86 but not for x64 since it's
    # enabled for x64 by default. The option will be ignored on x64 with a
    # warning message. If anyone knows how to detect the system's architecture
    # in CMake time, then I would gratefully apply these options differently
    # depending on the architectures.
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    list(APPEND compile_options_public -march=native -faligned-new)
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    list(APPEND compile_options_public -march=native -faligned-new)
  endif()
endif()

# Active log level
set(compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=${DART_ACTIVE_LOG_LEVEL})

# Explicit template instantiation options
if(DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE)
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_DOUBLE=1)
else()
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_DOUBLE=0)
endif()
if(DART_BUILD_TEMPLATE_CODE_FOR_FLOAT)
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_FLOAT=1)
else()
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_FLOAT=0)
endif()

# CodeCov settings
# Reference: https://github.com/codecov/example-cpp11-cmake
if(DART_CODECOV AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
    list(APPEND link_options_public --coverage)
  else()
    list(APPEND link_libraries_public_skip_checking --coverage)
  endif()
  list(APPEND compile_options_public
    -O0         # no optimization
    -g          # generate debug info
    --coverage  # sets all required flags
  )
  list(APPEND link_libraries_public_skip_checking gcov)
endif()

set(compile_definitions_public
  -DDART_ACTIVE_LOG_LEVEL=${DART_ACTIVE_LOG_LEVEL}
)
if(DART_ENABLE_THREAD_SAFE)
  list(APPEND compile_definitions_public -DDART_ENABLE_THREAD_SAFE=1)
else()
  list(APPEND compile_definitions_public -DDART_ENABLE_THREAD_SAFE=0)
endif()

# Add component
dart_add_component(
  PROJECT_NAME
    ${PROJECT_NAME}
  PROJECT_VERSION_MAJOR
    ${DART_MAJOR_VERSION}
  PROJECT_SOURCE_DIR
    ${DART_SOURCE_DIR}
  PROJECT_BINARY_DIR
    ${DART_BINARY_DIR}
  COMPONENT_NAME
    ${component_name}
  COMPONENT_DEPENDENCY_PACKAGES
    Boost Eigen3
  COMPONENT_DEPENDENCY_PACKAGES_OPTIONAL
    fmt spdlog
  TARGET_LINK_LIBRARIES_PUBLIC
    ${link_libraries_public}
  TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    ${CMAKE_DL_LIBS}
    $<$<PLATFORM_ID:Linux>:stdc++fs>
    ${link_libraries_public_skip_checking}
  TARGET_LINK_OPTIONS_PUBLIC
    ${link_options_public}
  TARGET_COMPILE_DEFINITIONS_PUBLIC
    ${compile_definitions_public}
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  TARGET_COMPILE_OPTIONS_PUBLIC
    ${compile_options_public}
  SUB_DIRECTORIES
    detail
  GENERATE_META_HEADER
  GENERATE_EXPORT_HEADER
  FORMAT_CODE
)
