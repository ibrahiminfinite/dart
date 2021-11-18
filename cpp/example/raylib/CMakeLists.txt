# Copyright (c) 2011-2021, The DART development contributors

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

dart_build_example(
  TARGET_NAME
    EXAMPLE_${example_name}
  DEPENDENT_COMPONENTS
    common
)
target_include_directories(EXAMPLE_${example_name} PUBLIC ${RAYLIB_INCLUDE_DIRS})
target_link_libraries(EXAMPLE_${example_name} PUBLIC ${RAYLIB_LIBRARIES})
