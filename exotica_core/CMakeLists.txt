cmake_minimum_required(VERSION 3.0.2)
project(exotica_core)

## catkin dependencies
set(CATKIN_DEPENDS
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

## system dependencies
set(SYSTEM_DEPENDS
  Boost
  EIGEN3
  TinyXML2
  ZeroMQ
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS cmake_modules ${CATKIN_DEPENDS})
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(ZeroMQ REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MSGPACK QUIET msgpack)
pkg_check_modules(TinyXML2 REQUIRED tinyxml2)

if(MSGPACK_FOUND)
  add_definitions(-DMSGPACK_FOUND)
  list(APPEND SYSTEM_DEPENDS MSGPACK)
else()
  message(STATUS "MessagePack not found. VisualizationMeshcat will be disabled.")
endif()

# NB: VERSION_GREATER_EQUAL requires CMake 3.7
if("${TinyXML2_VERSION}" VERSION_GREATER "5.90.0")
  add_definitions(-DTINYXML_HAS_ERROR_STR)
endif()

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
  dynamic_time_indexed_shooting_problem
  dynamics_solver
  collision_scene

  shape
  box_shape
  cylinder_shape
  mesh_shape
  octree_shape
  sphere_shape
)
GenInitializers()

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
  CFG_EXTRAS exotica.cmake add_initializer.cmake
  DEPENDS ${SYSTEM_DEPENDS}
)

## Build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}
  ${MSGPACK_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/task_map.cpp
  src/tools.cpp
  src/planning_problem.cpp
  src/motion_solver.cpp
  src/setup.cpp
  src/server.cpp
  src/visualization_moveit.cpp
  src/visualization_meshcat.cpp
  src/scene.cpp
  src/collision_scene.cpp
  src/trajectory.cpp
  src/property.cpp
  src/kinematic_tree.cpp
  src/dynamics_solver.cpp
  src/task_space_vector.cpp
  src/tools/exception.cpp
  src/tools/printable.cpp
  src/tools/conversions.cpp
  src/loaders/xml_loader.cpp
  src/tasks.cpp

  src/problems/abstract_time_indexed_problem.cpp
  src/problems/time_indexed_problem.cpp
  src/problems/bounded_time_indexed_problem.cpp
  src/problems/unconstrained_time_indexed_problem.cpp

  src/problems/end_pose_problem.cpp
  src/problems/bounded_end_pose_problem.cpp
  src/problems/unconstrained_end_pose_problem.cpp

  src/problems/sampling_problem.cpp
  src/problems/time_indexed_sampling_problem.cpp

  src/problems/dynamic_time_indexed_shooting_problem.cpp

  ${exotica_core_BINARY_DIR}/generated/version.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML2_LIBRARIES} ${ZeroMQ_LIBRARIES} ${MSGPACK_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})
# mark all warnings as errors
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra) # -Werror
# enable additional warnings
target_compile_options(${PROJECT_NAME} PRIVATE -Wdeprecated-declarations -Woverloaded-virtual -Wfloat-conversion)
# These work too: -Wstrict-overflow=5 -Wswitch-default -Wundef)

## Quick fixes to get Noetic to compile:
# - ZMQ is using deprecated send() declarations - remove once fixed for Noetic
# - -Wno-deprecated-copy needs to be appended here to avoid issues with pragmas for Melodic
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations) # -Wno-deprecated-copy)

# TODO: if you have time, remove these ignored warnings and fix them
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-parameter -Wno-sign-compare -Wno-reorder)

# Ignore Eigen::Tensor warnings
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-ignored-attributes)

# TODO: if you have even more time, uncomment these warnings and fix them :-)
# https://stackoverflow.com/a/9862800
# target_compile_options(${PROJECT_NAME} PRIVATE -pedantic -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-declarations -Wmissing-include-dirs -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel)

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY cmake/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
install(FILES cmake/initializer_project_header.h.in DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
install(PROGRAMS cmake/generate_initializers.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_autodiff test/test_autodiff.cpp)
  target_link_libraries(test_autodiff ${catkin_LIBRARIES} ${PROJECT_NAME})
  add_dependencies(test_autodiff ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${catkin_LIBRARIES} ${PROJECT_NAME})
  add_dependencies(test_kinematics ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()
