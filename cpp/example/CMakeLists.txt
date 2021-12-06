# Copyright (c) 2011-2021, The DART development contributors

# Automatically identify all directories in the apps folder
file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
set(directories "")
foreach(child ${children})
  if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
    add_subdirectory(${child})
  endif()
endforeach(child)

# Add custom target to build all the examples as a single target
dart_property_get(DART_GLOBAL_PROPERTY_EXAMPLE_LIST example_targets)
add_custom_target(examples DEPENDS ${example_targets})
