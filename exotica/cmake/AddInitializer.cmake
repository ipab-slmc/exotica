cmake_minimum_required(VERSION 2.8)

set(_InitializerInputFiles)
set(_InitializerOutputFiles)
set(_InitializerCopyFiles)
set(_InitializerScriptDir ${CMAKE_CURRENT_LIST_DIR})
set(_InitializerSearchPaths ${CMAKE_INSTALL_PREFIX})
set(_InitializerDepends)
foreach(path ${CMAKE_PREFIX_PATH})
    set(_InitializerSearchPaths "${_InitializerSearchPaths}:${path}")
endforeach()

foreach(depend ${catkin_FIND_COMPONENTS})
  if(TARGET ${depend}_initializers)
    message(STATUS "Dependency on: '${depend}_initializers'")
    list(APPEND _InitializerDepends ${depend}_initializers ${_InitializerDepends})
  endif()
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
add_custom_command(
  OUTPUT ${_InitializerOutputFiles}
  COMMAND python ${_InitializerScriptDir}/GenerateInitializers.py exotica "${_InitializerSearchPaths}" ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION} ${_InitializerInputFiles} ${_InitializerOutputFiles}
  DEPENDS ${_InitializerInputFiles} ${_InitializerScriptDir}/GenerateInitializers.py
)
add_custom_target(${PROJECT_NAME}_initializers DEPENDS ${_InitializerOutputFiles})
#list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_initializers ${_InitializerDepends})
endmacro(GenInitializers)

#message(STATUS "CMake prefix: '${CMAKE_PREFIX_PATH}'")
#message(STATUS "CMake install prefix: '${CMAKE_INSTALL_PREFIX}'")
#message(STATUS "Python: '${CATKIN_PACKAGE_PYTHON_DESTINATION}'")
#message(STATUS "Devel: '${CATKIN_DEVEL_PREFIX}'")
#message(STATUS "Install: '${CATKIN_INSTALL_PREFIX}'")
#message(STATUS "Package lib: '${CATKIN_PACKAGE_LIB_DESTINATION}'")
#message(STATUS "Package bin: '${CATKIN_PACKAGE_BIN_DESTINATION}'")
#message(STATUS "Package python: '${CATKIN_PACKAGE_PYTHON_DESTINATION}'")
#message(STATUS "Package include: '${CATKIN_GLOBAL_INCLUDE_DESTINATION}'")
#message(STATUS "Package share: '${CATKIN_PACKAGE_SHARE_DESTINATION}'")
#message(STATUS "Bin global: '${CATKIN_GLOBAL_BIN_DESTINATION}'")
#message(STATUS "Include: '${CATKIN_PACKAGE_INCLUDE_DESTINATION}'")

