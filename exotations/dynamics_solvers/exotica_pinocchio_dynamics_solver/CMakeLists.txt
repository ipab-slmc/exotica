cmake_minimum_required(VERSION 3.22)
project(exotica_pinocchio_dynamics_solver)

find_package(ament_cmake REQUIRED)
find_package(exotica_core REQUIRED)
find_package(pinocchio REQUIRED)

AddInitializer(
  pinocchio_dynamics_solver
  pinocchio_gravity_compensation_dynamics_solver
)
GenInitializers()

add_library(${PROJECT_NAME} SHARED
  src/pinocchio_dynamics_solver.cpp
  src/pinocchio_dynamics_solver_assign_scene.cpp
  src/pinocchio_dynamics_solver_derivatives.cpp
  src/pinocchio_dynamics_solver_inverse_dynamics.cpp
  src/pinocchio_gravity_compensation_dynamics_solver.cpp
  src/pinocchio_gravity_compensation_dynamics_solver_assign_scene.cpp
  src/pinocchio_gravity_compensation_dynamics_solver_derivatives.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-deprecated -Wno-variadic-macros -Wno-deprecated-declarations
  -Wno-comment -Wno-ignored-attributes)
ament_target_dependencies(${PROJECT_NAME} PUBLIC exotica_core)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(exotica_core exotica_plugins.xml)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(exotica_core pinocchio)
ament_package()
