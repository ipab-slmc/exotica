cmake_minimum_required(VERSION 2.8.3)
project(kinematica)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#add_definitions(-DKIN_DEBUG_MODE)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)
find_package(orocos_kdl REQUIRED)

if(EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/Ref.h")
   message(WARNING "Eigen found")
elseif(IS_DIRECTORY $ENV{DRC_BASE})
  set(EIGEN3_INCLUDE_DIR $ENV{DRC_BASE}/software/build/include/eigen3)
  message(WARNING "Using OpenHumanoids version of Eigen")
elseif(EXISTS "/usr/include/eigen3/Eigen/src/Core/Ref.h")
  message(WARNING "Manually setting to /usr/include/eigen3")
  set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
else()
   message(FATAL_ERROR "Eigen version 3.2.1 or higher is required! Install it in the standard directory.")
endif()

catkin_package(
 INCLUDE_DIRS  ${EIGEN3_INCLUDE_DIR} include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS kdl_parser
 DEPENDS system_lib boost orocos_kdl eigen
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  SYSTEM  
)

add_library(${PROJECT_NAME} src/KinematicTree.cpp src/tinyxml2.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
