# Copyright (c) 2011-2021, The DART development contributors

# Set up component
dart_component_setup(
  COMPONENT_NAME
    gui
  DEPENDENT_COMPONENTS
    common math simulation
  DEPENDENT_PACKAGES_REQUIRED
    raylib
  TARGET_LINK_LIBRARIES_PUBLIC
    raylib
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
  SUB_DIRECTORIES
    scene
)
