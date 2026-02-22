# add_initializer.cmake — ament-compatible rewrite
# Provides: AddInitializer(<name> ...) and GenInitializers()
#
# Usage in a downstream package:
#   find_package(exotica_core REQUIRED)
#   AddInitializer(foo bar)
#   GenInitializers()
#   add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

cmake_minimum_required(VERSION 3.22)

# Save the directory where this file lives (CMAKE_CURRENT_LIST_DIR is accurate
# here at file-processing time, but NOT inside macro bodies which run in the
# caller's scope).
set(_EXOTICA_INITIALIZER_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL "")

set(_EXOTICA_INITIALIZER_FILES "" CACHE INTERNAL "")

# Build the list of search directories from AMENT_PREFIX_PATH.
# Called once per CMake run; result stored in _EXOTICA_INIT_SEARCH_DIRS.
macro(_exotica_build_search_dirs)
  set(_EXOTICA_INIT_SEARCH_DIRS "")
  # Include CMAKE_CURRENT_BINARY_DIR so the package can find its own .in files
  # after generate_initializers.py copies them to share/${PROJECT_NAME}/init/
  list(APPEND _EXOTICA_INIT_SEARCH_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
  # Include CMAKE_INSTALL_PREFIX for packages already installed
  list(APPEND _EXOTICA_INIT_SEARCH_DIRS "${CMAKE_INSTALL_PREFIX}")
  if(DEFINED ENV{AMENT_PREFIX_PATH})
    string(REPLACE ":" ";" _ament_prefix_list "$ENV{AMENT_PREFIX_PATH}")
    foreach(_prefix ${_ament_prefix_list})
      list(APPEND _EXOTICA_INIT_SEARCH_DIRS "${_prefix}")
    endforeach()
  endif()
endmacro()

macro(AddInitializer)
  foreach(_init_name ${ARGN})
    # First look in the current package's own init/ directory
    set(_in_file "${CMAKE_CURRENT_SOURCE_DIR}/init/${_init_name}.in")
    if(EXISTS "${_in_file}")
      list(APPEND _EXOTICA_INITIALIZER_FILES "${_in_file}")
      message(STATUS "AddInitializer: found ${_init_name} locally")
    else()
      # Fall back: search installed share dirs
      _exotica_build_search_dirs()
      set(_found FALSE)
      foreach(_prefix ${_EXOTICA_INIT_SEARCH_DIRS})
        file(GLOB _candidates "${_prefix}/share/*/init/${_init_name}.in")
        if(_candidates)
          list(GET _candidates 0 _in_file)
          list(APPEND _EXOTICA_INITIALIZER_FILES "${_in_file}")
          message(STATUS "AddInitializer: found ${_init_name} at ${_in_file}")
          set(_found TRUE)
          break()
        endif()
      endforeach()
      if(NOT _found)
        message(FATAL_ERROR "AddInitializer: cannot find ${_init_name}.in "
          "(searched ${CMAKE_CURRENT_SOURCE_DIR}/init/ and share/*/init/ "
          "under AMENT_PREFIX_PATH)")
      endif()
    endif()
  endforeach()
endmacro()

macro(GenInitializers)
  # Locate generate_initializers.py.
  # _EXOTICA_INITIALIZER_CMAKE_DIR was captured when this .cmake file was
  # first processed (before any macro calls), so it reliably points to the
  # cmake/ directory regardless of which package calls GenInitializers().
  set(_gen_script "${_EXOTICA_INITIALIZER_CMAKE_DIR}/generate_initializers.py")
  if(NOT EXISTS "${_gen_script}")
    # Downstream packages after install: script lives in exotica_core's share
    if(exotica_core_DIR)
      # exotica_core_DIR is typically <prefix>/share/exotica_core/cmake/
      get_filename_component(_ec_share_dir "${exotica_core_DIR}" DIRECTORY)
      set(_gen_script "${_ec_share_dir}/cmake/generate_initializers.py")
    endif()
  endif()
  if(NOT EXISTS "${_gen_script}")
    message(FATAL_ERROR "GenInitializers: cannot locate generate_initializers.py "
      "(tried ${_EXOTICA_INITIALIZER_CMAKE_DIR}/generate_initializers.py)")
  endif()

  # Build colon-separated search path string for the Python script
  _exotica_build_search_dirs()
  string(REPLACE ";" ":" _search_path_str "${_EXOTICA_INIT_SEARCH_DIRS}")

  set(_out_dir "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}")
  set(_devel_dir "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}")
  file(MAKE_DIRECTORY "${_out_dir}")
  file(MAKE_DIRECTORY "${_devel_dir}/init")

  # Build lists of output header paths
  set(_output_headers "")
  foreach(_in_file ${_EXOTICA_INITIALIZER_FILES})
    get_filename_component(_name "${_in_file}" NAME_WE)
    list(APPEND _output_headers "${_out_dir}/${_name}_initializer.h")
  endforeach()

  # The numerator header (aggregates all initializers for this package)
  set(_numerator_header
    "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_initializers_numerator.h")

  # Single custom_command that calls generate_initializers.py for all .in files at once
  add_custom_command(
    OUTPUT ${_output_headers} ${_numerator_header}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_out_dir}"
    COMMAND python3 "${_gen_script}"
      "${PROJECT_NAME}"
      "${_search_path_str}"
      "${_devel_dir}"
      "${_numerator_header}"
      ${_EXOTICA_INITIALIZER_FILES}
      ${_output_headers}
    DEPENDS ${_EXOTICA_INITIALIZER_FILES} "${_gen_script}"
    COMMENT "Generating EXOTica initializers for ${PROJECT_NAME}"
  )

  add_custom_target(${PROJECT_NAME}_initializers
    DEPENDS ${_output_headers} ${_numerator_header}
  )

  # Make generated headers visible during build.
  # Use include_directories() so this works whether or not the target has been
  # created yet (downstream packages call GenInitializers() before add_library).
  # Each package is expected to also add ${CMAKE_CURRENT_BINARY_DIR}/include to
  # its own target_include_directories for proper transitive export.
  include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

  install(DIRECTORY "${_out_dir}/"
    DESTINATION "include/${PROJECT_NAME}"
    FILES_MATCHING PATTERN "*_initializer.h"
  )
  install(FILES "${_numerator_header}"
    DESTINATION "include/${PROJECT_NAME}"
    OPTIONAL
  )
  install(FILES ${_EXOTICA_INITIALIZER_FILES}
    DESTINATION "share/${PROJECT_NAME}/init"
  )
endmacro()
