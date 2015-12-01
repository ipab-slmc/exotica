cmake_minimum_required(VERSION 2.8.3)
project(drake_ik_filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)

find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drake_ik_filter
  CATKIN_DEPENDS exotica roscpp
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(drake_ik_filter
  src/DrakeIKFilter.cpp
)
pods_use_pkg_config_packages(drake_ik_filter drake-rbm drake-ik)
target_link_libraries(drake_ik_filter
  ${catkin_LIBRARIES} drakeRBM drakeIK
)