# Copyright (c) 2011-2021, The DART development contributors

if(DART_USE_EXTERNAL_fmt)
  return()
endif()

set(target_name fmt)

file(GLOB_RECURSE sources "*.h")

add_library(${target_name} INTERFACE ${sources})
add_library(fmt::fmt-header-only ALIAS ${target_name})

# Install targets
install(
  TARGETS ${target_name}
  EXPORT ${target_name}-targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
  EXPORT ${target_name}-targets
  FILE ${target_name}-targets.cmake
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}${DART_VERSION_MAJOR}
)
