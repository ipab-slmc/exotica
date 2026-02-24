cmake_minimum_required(VERSION 3.22)
project(exotica_ompl_solver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(exotica_core REQUIRED)
find_package(exotica_python REQUIRED)
find_package(ompl REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

_ament_cmake_python_get_python_install_dir()

AddInitializer(
  ompl_solver
  rrt
  rrt_connect
  prm
  lazy_prm
  est
  kpiece
  bkpiece
  rrt_star
  lbt_rrt
)
GenInitializers()

add_library(${PROJECT_NAME} SHARED
  src/ompl_exo.cpp
  src/ompl_solver.cpp
  src/ompl_native_solvers.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC exotica_core)
target_link_libraries(${PROJECT_NAME} PUBLIC ompl::ompl)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

# Python bindings module
python3_add_library(${PROJECT_NAME}_py MODULE src/ompl_py.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_py ${PROJECT_NAME} ${PROJECT_NAME}_initializers)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_py
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}_py
)

ament_python_install_package(${PROJECT_NAME}_py
  PACKAGE_DIR src/${PROJECT_NAME}_py)

install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(exotica_core ompl)

pluginlib_export_plugin_description_file(exotica_core exotica_plugins.xml)

ament_package()
