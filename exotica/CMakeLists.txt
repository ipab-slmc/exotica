cmake_minimum_required(VERSION 2.8.3)
project(exotica)

## Set Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)

set(MSG_DEPS
  geometry_msgs
  std_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  kinematica
  moveit_ros_planning_interface
  moveit_core
  tf
  ${MSG_DEPS}
)

set(MSG_FILES
  MeshVertex.msg
  MeshVertexArray.msg
  StringList.msg
  BoolList.msg
  Vector.msg
  Matrix.msg
)

add_message_files(
  DIRECTORY
  FILES
  ${MSG_FILES}
)

generate_messages(
  DEPENDENCIES
  ${MSG_DEPS}
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

if(EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/Ref.h")
   message(STATUS "Eigen found")
else()
   message(FATAL_ERROR "Eigen version 3.2.1 or higher is required! Install it in the standard directory.")
endif()

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  message_runtime kinematica moveit_ros_planning_interface moveit_core tf
  DEPENDS eigen system_lib boost
)

## Build ##
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/TaskMap.cpp
  src/tinyxml2.cpp
  src/Tools.cpp 
  src/TaskDefinition.cpp 
  src/PlanningProblem.cpp 
  src/MotionSolver.cpp 
  src/Initialiser.cpp 
  src/Server.cpp
  src/Scene.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
