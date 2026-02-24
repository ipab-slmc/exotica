cmake_minimum_required(VERSION 3.22)
project(exotica_ilqg_solver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(exotica_core REQUIRED)
find_package(exotica_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

_ament_cmake_python_get_python_install_dir()

AddInitializer(ilqg_solver)
GenInitializers()

add_library(${PROJECT_NAME} SHARED src/ilqg_solver.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC exotica_core)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

python3_add_library(${PROJECT_NAME}_py MODULE src/ilqg_solver_py.cpp)
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
ament_export_dependencies(exotica_core)

pluginlib_export_plugin_description_file(exotica_core exotica_plugins.xml)

ament_package()
