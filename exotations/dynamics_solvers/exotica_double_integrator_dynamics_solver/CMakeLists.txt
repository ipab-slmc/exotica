cmake_minimum_required(VERSION 3.22)
project(exotica_double_integrator_dynamics_solver)

find_package(ament_cmake REQUIRED)
find_package(exotica_core REQUIRED)

AddInitializer(double_integrator_dynamics_solver)
GenInitializers()

add_library(${PROJECT_NAME} SHARED src/double_integrator_dynamics_solver.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC exotica_core)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(exotica_core exotica_plugins.xml)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(exotica_core)
ament_package()
