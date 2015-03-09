cmake_minimum_required(VERSION 2.8.3)
project(kinematica)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#add_definitions(-DKIN_DEBUG_MODE)
add_definitions(-DEXOTICA_DEBUG_MODE)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)
find_package(orocos_kdl REQUIRED)
##find_package(EIGEN REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS kdl_parser
 DEPENDS system_lib boost orocos_kdl eigen
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/KinematicTree.cpp src/tinyxml2.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${orocos_kdl_LIBRARIES})
