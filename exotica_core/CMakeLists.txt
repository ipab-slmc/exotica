cmake_minimum_required(VERSION 2.8.12)
project(exotica_core)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_msgs
  tf
  eigen_conversions
  kdl_parser
  pluginlib
  tf_conversions
  geometry_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS signals)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(TinyXML2 REQUIRED)

include(exotica)
include(add_initializer)

include(get_git_revision_description)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
configure_file("src/version.cpp.in" "${exotica_core_BINARY_DIR}/generated/version.cpp" @ONLY)

AddInitializer(
  task_map
  object
  frame
  link
  attach_link
  motion_solver
  planning_problem
  scene
  trajectory
  unconstrained_time_indexed_problem
  unconstrained_end_pose_problem
  sampling_problem
  time_indexed_sampling_problem
  task
  time_indexed_problem
  end_pose_problem
  bounded_time_indexed_problem
  bounded_end_pose_problem
)
GenInitializers()

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} include ${Boost_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime moveit_core moveit_ros_planning tf kdl_parser pluginlib tf_conversions eigen_conversions
  CFG_EXTRAS exotica.cmake add_initializer.cmake
  DEPENDS TinyXML2
)

## Build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/task_map.cpp
  src/tools.cpp
  src/planning_problem.cpp
  src/motion_solver.cpp
  src/setup.cpp
  src/server.cpp
  src/visualization.cpp
  src/scene.cpp
  src/trajectory.cpp
  src/property.cpp
  src/kinematic_tree.cpp
  src/task_space_vector.cpp
  src/tools/exception.cpp
  src/tools/printable.cpp
  src/tools/conversions.cpp
  src/loaders/xml_loader.cpp
  src/tasks.cpp
  src/problems/unconstrained_time_indexed_problem.cpp
  src/problems/unconstrained_end_pose_problem.cpp
  src/problems/time_indexed_problem.cpp
  src/problems/end_pose_problem.cpp
  src/problems/bounded_time_indexed_problem.cpp
  src/problems/bounded_end_pose_problem.cpp
  src/problems/sampling_problem.cpp
  src/problems/time_indexed_sampling_problem.cpp
  ${exotica_core_BINARY_DIR}/generated/version.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML2_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY cmake/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
install(FILES cmake/generate_initializers.py cmake/initializer_project_header.h.in DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_autodiff test/test_autodiff.cpp)
  target_link_libraries(test_autodiff ${catkin_LIBRARIES} ${PROJECT_NAME})
  add_dependencies(test_autodiff ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()