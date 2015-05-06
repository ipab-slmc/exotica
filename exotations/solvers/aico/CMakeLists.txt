cmake_minimum_required(VERSION 2.8.3)
project(aico)

## Set Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
add_definitions(-DEXOTICA_DEBUG_MODE)  #Controls whether to compile with certain debug checks included (-DEXOTICA_DEBUG_MODE)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  roslib
  task_definition
  task_map
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aico
  CATKIN_DEPENDS exotica roscpp roslib task_definition task_map
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(aico
  src/AICOsolver.cpp
  src/AICOProblem.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(aico
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES}
)
