cmake_minimum_required(VERSION 2.8)

find_package(PythonInterp REQUIRED)

set(_InitializerInputFiles)
set(_InitializerOutputFiles)
set(_InitializerCopyFiles)
set(_InitializerScriptDir ${CMAKE_CURRENT_LIST_DIR})
set(_InitializerSearchPaths ${CMAKE_INSTALL_PREFIX}:${CATKIN_DEVEL_PREFIX})
foreach(path ${CMAKE_PREFIX_PATH})
    set(_InitializerSearchPaths "${_InitializerSearchPaths}:${path}")
endforeach()

macro(AddInitializer)
  foreach(arg ${ARGN})
    list(APPEND _InitializerInputFiles ${CMAKE_CURRENT_SOURCE_DIR}/init/${arg}.in)
    list(APPEND _InitializerOutputFiles ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${arg}Initializer.h)
    list(APPEND _InitializerCopyFiles ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/init/${arg}.in)
    message(STATUS "Adding initializer: '${arg}'")
  endforeach()
endmacro(AddInitializer)

macro(GenInitializers)
  include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  add_custom_command(
    OUTPUT ${_InitializerOutputFiles}
    COMMAND ${PYTHON_EXECUTABLE} ${_InitializerScriptDir}/GenerateInitializers.py exotica "${_InitializerSearchPaths}" ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION} ${_InitializerInputFiles} ${_InitializerOutputFiles}
    DEPENDS ${_InitializerInputFiles} ${_InitializerScriptDir}/GenerateInitializers.py
  )
  add_custom_target(${PROJECT_NAME}_initializers DEPENDS ${_InitializerOutputFiles})

  install(FILES ${_InitializerOutputFiles}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(FILES ${_InitializerInputFiles}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/init)
endmacro(GenInitializers)
