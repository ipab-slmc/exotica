cmake_minimum_required(VERSION 2.8.12)
project(exotica)

set(MSG_DEPS
  geometry_msgs
  std_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  moveit_core
  moveit_ros_planning
  tf
  eigen_conversions
  kdl_parser
  pluginlib
  tf_conversions
  ${MSG_DEPS}
)
find_package(Boost REQUIRED COMPONENTS signals)

set(MSG_FILES
  MeshVertex.msg
  MeshVertexArray.msg
  Vector.msg
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


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(Exotica)
include(AddInitializer)

include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
configure_file("src/Version.cpp.in" "${exotica_BINARY_DIR}/generated/Version.cpp" @ONLY)

AddInitializer(
  TaskMap
  Object
  Frame
  Link
  AttachLink
  MotionSolver
  PlanningProblem
  Scene
  Trajectory
  UnconstrainedTimeIndexedProblem
  UnconstrainedEndPoseProblem
  SamplingProblem
  TimeIndexedSamplingProblem
  Task
  TimeIndexedProblem
  EndPoseProblem
  BoundedTimeIndexedProblem
  BoundedEndPoseProblem
)
GenInitializers()

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime moveit_core moveit_ros_planning tf kdl_parser pluginlib tf_conversions eigen_conversions
  CFG_EXTRAS Exotica.cmake AddInitializer.cmake
)

## Build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/TaskMap.cpp
  src/Loaders/tinyxml2.cpp
  src/Tools.cpp
  src/PlanningProblem.cpp
  src/MotionSolver.cpp
  src/Setup.cpp
  src/Server.cpp
  src/Scene.cpp
  src/Trajectory.cpp
  src/Property.cpp
  src/KinematicTree.cpp
  src/TaskSpaceVector.cpp
  src/Tools/Exception.cpp
  src/Tools/Printable.cpp
  src/Tools/Conversions.cpp
  src/Loaders/XMLLoader.cpp
  src/Tasks.cpp
  src/Problems/UnconstrainedTimeIndexedProblem.cpp
  src/Problems/UnconstrainedEndPoseProblem.cpp
  src/Problems/TimeIndexedProblem.cpp
  src/Problems/EndPoseProblem.cpp
  src/Problems/BoundedTimeIndexedProblem.cpp
  src/Problems/BoundedEndPoseProblem.cpp
  src/Problems/SamplingProblem.cpp
  src/Problems/TimeIndexedSamplingProblem.cpp
  ${exotica_BINARY_DIR}/generated/Version.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_initializers)

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY cmake/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
install(FILES cmake/GenerateInitializers.py cmake/InitializerProjectHeader.h.in DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)

