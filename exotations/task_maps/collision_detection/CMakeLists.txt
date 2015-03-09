cmake_minimum_required(VERSION 2.8.3)
project(collision_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
add_definitions(-DEXOTICA_DEBUG_MODE)

find_package(catkin REQUIRED COMPONENTS
  exotica
  kinematica
  moveit_core
  moveit_msgs
  eigen_conversions
  fcl
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica kinematica moveit_core moveit_msgs eigen_conversions fcl
  DEPENDS 
)

include_directories(
  include
  fcl
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
	src/exotica_collision.cpp
	src/collision_detection_exotica/collision_robot_exotica.cpp
	src/collision_detection_exotica/collision_world_exotica.cpp
	src/collision_detection_exotica/collision_common.cpp
           )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})



