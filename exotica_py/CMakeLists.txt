cmake_minimum_required(VERSION 2.8.3)
project(exotica_py)

find_package(catkin REQUIRED COMPONENTS
  exotica
  pybind11
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES exotica_py
  CATKIN_DEPENDS exotica pybind11
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

pybind_add_module(exotica_py MODULE src/Exotica.cpp)
