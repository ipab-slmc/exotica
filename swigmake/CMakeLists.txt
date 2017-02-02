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

## Install
install(DIRECTORY swigmake/swig/matlab DESTINATION share/swigmake/swig/)
install(DIRECTORY swigmake/swig/python DESTINATION share/swigmake/swig/)
install(DIRECTORY swigmake/swig/common DESTINATION share/swigmake/swig/)
install(DIRECTORY swigmake/cmake/ DESTINATION share/swigmake/cmake/)
