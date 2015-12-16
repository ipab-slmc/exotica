cmake_minimum_required(VERSION 2.8.3)
project(exotica_distro)

find_package(catkin REQUIRED COMPONENTS
  exotica
  aico
  dmesh_ros
  ik_solver
  kinematica
  ompl_solver
  task_definition
  task_map
)

catkin_package(
  CATKIN_DEPENDS aico dmesh_ros exotica ik_solver kinematica ompl_solver task_definition task_map
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
