cmake_minimum_required(VERSION 2.8.3)
project(exotica_python)

find_package(catkin REQUIRED COMPONENTS
  exotica
  pybind11
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS exotica pybind11
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

pybind_add_module(pyexotica MODULE src/Exotica.cpp)
install(TARGETS pyexotica LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
