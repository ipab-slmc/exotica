cmake_minimum_required(VERSION 2.8.3)
project(exotica)

## Set Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
#add_definitions(-DEXOTICA_DEBUG_MODE)  #Controls whether to compile with certain debug checks included (-DEXOTICA_DEBUG_MODE)

set(MSG_DEPS
  geometry_msgs
  std_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  kinematic_scene
  roscpp
  message_generation
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
##find_package(EIGEN REQUIRED)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS kinematic_scene roscpp
  DEPENDS eigen system_lib boost message_runtime
)

## Build ##
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES})
