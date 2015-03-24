cmake_minimum_required(VERSION 2.8.3)
project(aico)

## Set Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
add_definitions(-DEXOTICA_DEBUG_MODE)  #Controls whether to compile with certain debug checks included (-DEXOTICA_DEBUG_MODE)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  exotica
  kinematic_scene
  roscpp
  roslib
  task_definition
  task_map
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)

## System dependencies are found with CMake's conventions
##find_package(EIGEN REQUIRED)
find_package(LAPACK REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aico
  CATKIN_DEPENDS exotica roscpp roslib task_definition kinematic_scene task_map
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(aico
  src/AICOsolver.cpp
  src/AICOProblem.cpp
)

add_executable(example
	src/AICOexample.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(aico
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${LAPACK_LIBRARIES}
)

target_link_libraries(example
  ${catkin_LIBRARIES} aico)
