cmake_minimum_required(VERSION 3.22)
project(exotica_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(geometric_shapes REQUIRED)

# Generate version.cpp from template
configure_file(src/version.cpp.in src/version.cpp @ONLY)

set(SOURCES
  src/collision_scene.cpp
  src/dynamics_solver.cpp
  src/kinematic_tree.cpp
  src/loaders/xml_loader.cpp
  src/motion_solver.cpp
  src/planning_problem.cpp
  src/problems/abstract_time_indexed_problem.cpp
  src/problems/bounded_end_pose_problem.cpp
  src/problems/bounded_time_indexed_problem.cpp
  src/problems/dynamic_time_indexed_shooting_problem.cpp
  src/problems/end_pose_problem.cpp
  src/problems/sampling_problem.cpp
  src/problems/time_indexed_problem.cpp
  src/problems/time_indexed_sampling_problem.cpp
  src/problems/unconstrained_end_pose_problem.cpp
  src/problems/unconstrained_time_indexed_problem.cpp
  src/property.cpp
  src/scene.cpp
  src/server.cpp
  src/setup.cpp
  src/task_map.cpp
  src/task_space_vector.cpp
  src/tasks.cpp
  src/tools.cpp
  src/tools/conversions.cpp
  src/tools/exception.cpp
  src/tools/printable.cpp
  src/trajectory.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/src/version.cpp
  src/visualization_meshcat.cpp
  src/visualization_moveit.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${MSGPACK_INCLUDE_DIR}
    ${ZMQ_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_msgs
  tf2_ros
  tf2_eigen_kdl
  tf2_kdl
  kdl_parser
  pluginlib
  geometry_msgs
  std_msgs
  visualization_msgs
  octomap_msgs
  shape_msgs
  sensor_msgs
  orocos_kdl_vendor
  geometric_shapes
)
target_link_libraries(${PROJECT_NAME}
  ${ZMQ_LIBRARIES}
  tinyxml2::tinyxml2
)

# Include the add_initializer.cmake macros and generate this package's initializers
include(cmake/add_initializer.cmake)

AddInitializer(
  attach_link
  bounded_end_pose_problem
  bounded_time_indexed_problem
  box_shape
  collision_scene
  cylinder_shape
  dynamic_time_indexed_shooting_problem
  dynamics_solver
  end_pose_problem
  frame
  link
  mesh_shape
  motion_solver
  object
  octree_shape
  planning_problem
  sampling_problem
  scene
  shape
  sphere_shape
  task
  task_map
  time_indexed_problem
  time_indexed_sampling_problem
  trajectory
  unconstrained_end_pose_problem
  unconstrained_time_indexed_problem
)
GenInitializers()
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY init/ DESTINATION share/${PROJECT_NAME}/init)
install(FILES
  cmake/exotica.cmake
  cmake/add_initializer.cmake
  cmake/generate_initializers.py
  DESTINATION share/${PROJECT_NAME}/cmake
)
# Install generated initializer headers so downstream packages can use them
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp ament_index_cpp moveit_core moveit_ros_planning moveit_msgs
  tf2_ros tf2_eigen_kdl tf2_kdl kdl_parser pluginlib
  geometry_msgs std_msgs visualization_msgs octomap_msgs
  shape_msgs sensor_msgs orocos_kdl_vendor geometric_shapes
)

# Export cmake extras so downstream packages get AddInitializer / GenInitializers
ament_package(CONFIG_EXTRAS cmake/exotica.cmake cmake/add_initializer.cmake)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_autodiff test/test_autodiff.cpp)
  target_link_libraries(test_autodiff ${PROJECT_NAME})

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})

  # test_problems and test_random_state (if present)
endif()
