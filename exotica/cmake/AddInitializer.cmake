cmake_minimum_required(VERSION 2.8)

set(_InitializerInputFiles)
set(_InitializerOutputFiles)
get_filename_component(_InitializerScriptDir ${CMAKE_CURRENT_LIST_FILE} PATH)

macro(AddInitializer)
  foreach(arg ${ARGN})
    list(APPEND _InitializerInputFiles ${arg}.in)
    list(APPEND _InitializerOutputFiles ${arg}.h)
  endforeach()
endmacro(AddInitializer)

macro(GenInitializers)
add_custom_command(
  OUTPUT ${_InitializerOutputFiles}
  COMMAND python ${_InitializerScriptDir}/GenerateInitializers.py ${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR} ${_InitializerInputFiles}
  DEPENDS ${_InitializerInputFiles} ${_InitializerScriptDir}/GenerateInitializers.py
)
add_custom_target(${PROJECT_NAME}_initializers DEPENDS ${_InitializerOutputFiles})
endmacro(GenInitializers)
