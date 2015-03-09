cmake_minimum_required(VERSION 2.8.3)
project(task_map)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
add_definitions(-DEXOTICA_DEBUG_MODE)

find_package(catkin REQUIRED COMPONENTS
  exotica
  kinematica
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica kinematica geometry_msgs
  DEPENDS #system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/kinematic_maps/CoM.cpp
                            src/kinematic_maps/IMesh.cpp
                            #src/kinematic_maps/DMesh.cpp
                            src/kinematic_maps/EffPosition.cpp
                            src/kinematic_maps/JointLimit.cpp
                            src/kinematic_maps/SweepFlux.cpp
                            src/kinematic_maps/Distance.cpp
                            src/kinematic_maps/Orientation.cpp
                            src/generic/Identity.cpp
           )
           

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
