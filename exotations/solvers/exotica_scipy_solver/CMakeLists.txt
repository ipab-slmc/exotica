cmake_minimum_required(VERSION 3.22)
project(exotica_scipy_solver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

ament_package()
