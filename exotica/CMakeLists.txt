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
  moveit_ros_planning_interface
  moveit_core
  tf
  kdl_parser
  ${MSG_DEPS}
)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs REQUIRED)
find_package(orocos_kdl REQUIRED)

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

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  message_runtime moveit_ros_planning_interface moveit_core tf kdl_parser
  DEPENDS eigen system_lib boost orocos_kdl
)

## Build ##
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  SYSTEM
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
  src/Scene.cpp
  src/KinematicTree.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
