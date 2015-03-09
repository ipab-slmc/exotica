cmake_minimum_required(VERSION 2.8.3)
project(kinematic_scene)
add_definitions(-DEXOTICA_DEBUG_MODE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
find_package(catkin REQUIRED COMPONENTS
  kinematica
  moveit_ros_planning_interface
  moveit_core
  tf
  eigen_conversions
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinematic_scene
  CATKIN_DEPENDS kinematica moveit_ros_planning_interface moveit_core tf eigen_conversions
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/kinematic_scene.cpp )

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
