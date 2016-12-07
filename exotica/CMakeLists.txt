cmake_minimum_required(VERSION 2.8.3)
project(exotica)

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
  pluginlib
  ${MSG_DEPS}
)

find_package(Boost REQUIRED COMPONENTS python signals)

set(MSG_FILES
  MeshVertex.msg
  MeshVertexArray.msg
  StringList.msg
  BoolList.msg
  Vector.msg
  Matrix.msg
  Problem.msg
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

AddInitializer(TaskMap Object Limb MotionSolver PlanningProblem Scene TaskDefinition Kinematica
     AICOProblem IKProblem OMPLProblem TaskSqrError TaskTerminationCriterion)
GenInitializers()

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  message_runtime moveit_ros_planning_interface moveit_core tf kdl_parser pluginlib
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
  src/tinyxml2.cpp
  src/Tools.cpp
  src/TaskDefinition.cpp
  src/PlanningProblem.cpp
  src/MotionSolver.cpp
  src/Setup.cpp
  src/Server.cpp
  src/Scene.cpp
  src/Property.cpp
  src/KinematicTree.cpp
  src/Tools/Exception.cpp
  src/Tools/Printable.cpp
  src/Tools/Conversions.cpp
  src/Loaders/XMLLoader.cpp
  src/Problems/AICOProblem.cpp
  src/Problems/IKProblem.cpp
  src/Problems/OMPLProblem.cpp
  src/Definitions/TaskSqrError.cpp
  src/Definitions/TaskTerminationCriterion.cpp
  ${exotica_BINARY_DIR}/generated/Version.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${exotica_BINARY_DIR}/generated/Version.cpp ${PROJECT_NAME}_initializers)

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)


## Testing

if(CATKIN_ENABLE_TESTING)
    add_executable(test_initializers tests/test_initializers.cpp)
    target_link_libraries(test_initializers ${PROJECT_NAME} ${catkin_LIBRARIES})
    install(TARGETS test_initializers RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif(CATKIN_ENABLE_TESTING)
