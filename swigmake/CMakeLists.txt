cmake_minimum_required(VERSION 2.8.3)
project(swigmake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/swigmake/cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED
)

find_package(Boost REQUIRED COMPONENTS python signals)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  CFG_EXTRAS swigmake.cmake mex.cmake ../swigmake/cmake/SwigMatlab.cmake ../swigmake/cmake/SwigPython.cmake
)

add_custom_target(includes ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/swigmake/swig" "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/swig")

## Install
install(DIRECTORY swigmake/swig/matlab DESTINATION share/swigmake/swig/matlab)
install(DIRECTORY swigmake/swig/python DESTINATION share/swigmake/swig/python)
install(DIRECTORY swigmake/swig/common DESTINATION share/swigmake/swig/common)
install(DIRECTORY swigmake/cmake/ DESTINATION share/swigmake/cmake/)
