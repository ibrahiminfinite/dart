# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Check dependencies
if(NOT DOXYGEN_FOUND)
  message(WARNING "Disabled building documentation due to missing dependency, [Doxygen]")
  return()
endif()

if(NOT Sphinx_FOUND)
  message(WARNING "Disabled building documentation due to missing dependency, [Sphinx]")
  return()
endif()

# References:
#   http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
#   http://www.cmake.org/pipermail/cmake/2007-February/012796.html
set(DOXYGEN_DOXYFILE_IN      "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in"    )
set(DOXYGEN_DOXYFILE         "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"       )
set(DOXYGEN_HTML_INDEX       "${CMAKE_CURRENT_BINARY_DIR}/doxygen/html/index.html")
set(DOXYGEN_OUTPUT_ROOT      "${CMAKE_CURRENT_BINARY_DIR}/doxygen/html"   )
set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/${PROJECT_NAME}.tag" )
set(DOXYGEN_INCLUDE_PATH     "${PROJECT_SOURCE_DIR}"                      )
set(DOXYGEN_INPUT_ROOT       "${PROJECT_SOURCE_DIR}/dart"                 )
set(DOXYGEN_EXTRA_INPUTS     "${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox"   )
set(DOXYGEN_EXCLUDE          "${PROJECT_SOURCE_DIR}/dart/external"        )
set(DOXYGEN_STRIP_FROM_PATH  "${CMAKE_CURRENT_SOURCE_DIR}"                )

# Generate a Doxyfile. This uses the variables:
#
# - DOXYGEN_OUTPUT_ROOT
# - DOXYGEN_GENERATE_TAGFILE
# - DOXYGEN_EXTRA_INPUTS
# - DOXYGEN_INPUT_ROOT
# - DOXYGEN_EXCLUDE
# - DOXYGEN_STRIP_FROM_PATH
configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
file(
  COPY "${CMAKE_CURRENT_SOURCE_DIR}/logo/DART logo.png"
  DESTINATION ${DOXYGEN_OUTPUT_ROOT}
)
add_custom_command(
  OUTPUT ${DOXYGEN_HTML_INDEX}
  COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
  COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
  # Strip path prefix from all paths in dart.tag
  COMMAND ${CMAKE_COMMAND} -E echo "Stripping paths from"
      "${DOXYGEN_GENERATE_TAGFILE}"
  COMMAND sed -i s:${DOXYGEN_STRIP_FROM_PATH}::g ${DOXYGEN_GENERATE_TAGFILE}
  # Strip all doxygen="path" HTML tags
  COMMAND ${CMAKE_COMMAND} -E echo "Stripping Doxygen HTML tags"
  COMMAND find "${DOXYGEN_OUTPUT_ROOT}" -type f -name "*.html"
      -exec sed -i 's: doxygen=\"[^\"]*\"::g' {} \\$<SEMICOLON>
  COMMAND ${CMAKE_COMMAND} -E echo "Done."
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${DOXYGEN_DOXYFILE}
)

add_custom_target(doxygen DEPENDS ${DOXYGEN_HTML_INDEX})
add_custom_target(
  doxygen_forced
  COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
  COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
  COMMAND ${CMAKE_COMMAND} -E echo "Done."
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/conf.py.in ${CMAKE_CURRENT_BINARY_DIR}/conf.py @ONLY)

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set(SPHINX_HTML_INDEX ${SPHINX_BUILD}/index.html)
add_custom_target(sphinx
  COMMAND ${SPHINX_EXECUTABLE} -b html
    # Tell Breathe where to find the Doxygen output
    -Dbreathe_projects.DART=${CMAKE_CURRENT_BINARY_DIR}/doxygen/xml
    ${SPHINX_SOURCE} ${SPHINX_BUILD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating documentation with Sphinx"
)

# Add "docs" target that builds doxygen and sphinx
add_custom_target(docs DEPENDS doxygen sphinx)

# Add "docs_forced" target that builds doxygen and sphinx
add_custom_target(docs_forced DEPENDS doxygen_forced sphinx)
# TODO(JS): Add sphinx_forced target

# Add the "view_docs" target that opens the generated API documentation.
if(APPLE)
  set(OPEN_COMMAND "open")
else()
  set(OPEN_COMMAND "xdg-open")
endif()
add_custom_target(view_docs "${OPEN_COMMAND}" "${SPHINX_HTML_INDEX}"
  DEPENDS "${SPHINX_HTML_INDEX}"
  COMMENT "Opening documentation in a web browser."
)
