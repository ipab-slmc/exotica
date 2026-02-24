cmake_minimum_required(VERSION 3.22)
project(exotica_core_task_maps)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Ensure PYTHON_INSTALL_DIR is set (needed for pybind11 module install destination)
_ament_cmake_python_get_python_install_dir()
find_package(exotica_core REQUIRED)
find_package(exotica_python REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Collect all task map source files (excludes the python bindings)
file(GLOB TASK_MAP_SOURCES src/*.cpp)
list(REMOVE_ITEM TASK_MAP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/task_map_py.cpp)

AddInitializer(
  avoid_look_at_sphere
  center_of_mass
  collision_check
  collision_distance
  continuous_joint_pose
  control_regularization
  distance
  distance_to_line_2d
  eff_axis_alignment
  eff_box
  eff_frame
  eff_orientation
  eff_position
  eff_position_xy
  eff_velocity
  frame_with_axis_and_direction
  frame_with_box_limits
  gaze_at_constraint
  interaction_mesh
  joint_acceleration_backward_difference
  joint_jerk_backward_difference
  joint_limit
  joint_pose
  joint_torque_minimization_proxy
  joint_velocity_backward_difference
  joint_velocity_limit
  joint_velocity_limit_constraint
  look_at
  manipulability
  point_to_line
  point_to_plane
  quasi_static
  smooth_collision_distance
  sphere
  sphere_collision
  sum_of_penetrations
  variable_size_collision_distance
)
GenInitializers()

add_library(${PROJECT_NAME} SHARED ${TASK_MAP_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  exotica_core
  tf2_kdl
  tf2_eigen
  visualization_msgs
  geometry_msgs
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

# Python bindings module
pybind11_add_module(exotica_core_task_maps_py MODULE src/task_map_py.cpp)
ament_target_dependencies(exotica_core_task_maps_py PUBLIC exotica_core exotica_python)
target_link_libraries(exotica_core_task_maps_py PUBLIC ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS exotica_core_task_maps_py
  DESTINATION "${PYTHON_INSTALL_DIR}/exotica_core_task_maps_py"
)

ament_python_install_package(exotica_core_task_maps_py
  PACKAGE_DIR src/exotica_core_task_maps_py)

install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(exotica_core tf2_kdl tf2_eigen visualization_msgs geometry_msgs)

pluginlib_export_plugin_description_file(exotica_core exotica_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_maps test/test_maps.cpp)
  target_link_libraries(test_maps ${PROJECT_NAME})
endif()

ament_package()
