cmake_minimum_required(VERSION 3.22)
project(exotica_python)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(exotica_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Build the _pyexotica C extension
pybind11_add_module(_pyexotica MODULE src/pyexotica.cpp)
ament_target_dependencies(_pyexotica PUBLIC
  exotica_core
  geometry_msgs
  moveit_msgs
  shape_msgs
  rclcpp
  ament_index_cpp
)
target_compile_options(_pyexotica PRIVATE -Wno-deprecated-declarations)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS _pyexotica
  DESTINATION "${PYTHON_INSTALL_DIR}/pyexotica"
)

# Install the Python package
ament_python_install_package(pyexotica
  PACKAGE_DIR src/pyexotica
)

install(PROGRAMS scripts/convert_moveit_scene_to_sdf
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_no_unknown test/test_no_unknown_initializer_types.py)
  ament_add_pytest_test(test_mesh test/test_mesh.py)
  ament_add_pytest_test(test_sparse test/test_sparse_costs.py)
endif()

ament_package()
