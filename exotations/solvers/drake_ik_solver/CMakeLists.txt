cmake_minimum_required(VERSION 2.8.3)
project(drake_ik_solver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  roslib
)


find_package(Boost REQUIRED COMPONENTS signals)
#find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

catkin_package(
INCLUDE_DIRS include
LIBRARIES drake_ik_solver
CATKIN_DEPENDS exotica roscpp roslib  
DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(drake_ik_solver
  src/DrakeIKProblem.cpp
  src/DrakeIKSolver.cpp
)
pods_use_pkg_config_packages(drake_ik_solver drake-rbm drake-ik lcm)

target_link_libraries(drake_ik_solver
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} drakeRBM drakeIK
)