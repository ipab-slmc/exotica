cmake_minimum_required(VERSION 2.8.3)
project(drm_ik_solver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
find_package(catkin REQUIRED COMPONENTS
  drake_ik_solver
  ik_solver
  dynamic_reachability_map
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drm_ik_solver
  CATKIN_DEPENDS drake_ik_solver ik_solver dynamic_reachability_map
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(drm_ik_solver
  src/DRMDrakeIKSolver.cpp
  src/DRMDrakeIKProblem.cpp
)
pods_use_pkg_config_packages(drm_ik_solver drake-rbm drake-ik)

target_link_libraries(drm_ik_solver
  ${catkin_LIBRARIES} drakeRBM drakeIK
)
