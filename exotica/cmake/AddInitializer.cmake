cmake_minimum_required(VERSION 2.8)

set(_InitializerInputFiles)
set(_InitializerOutputFiles)
set(_InitializerScriptDir ${CMAKE_CURRENT_LIST_DIR})

macro(AddInitializer)
  foreach(arg ${ARGN})
    list(APPEND _InitializerInputFiles ${CMAKE_CURRENT_SOURCE_DIR}/init/${arg}.in)
    list(APPEND _InitializerOutputFiles ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${arg}Initializer.h)
    message(STATUS "Adding initializer: '${arg}'")
  endforeach()
endmacro(AddInitializer)

macro(GenInitializers)
add_custom_command(
  OUTPUT ${_InitializerOutputFiles}
  COMMAND python ${_InitializerScriptDir}/GenerateInitializers.py exotica ${_InitializerInputFiles} ${_InitializerOutputFiles}
  DEPENDS ${_InitializerInputFiles} ${_InitializerScriptDir}/GenerateInitializers.py
)
add_custom_target(${PROJECT_NAME}_initializers DEPENDS ${_InitializerOutputFiles})
endmacro(GenInitializers)
