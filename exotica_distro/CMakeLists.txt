cmake_minimum_required(VERSION 2.8.3)
project(exotica_distro)

find_package(catkin REQUIRED COMPONENTS
  exotica
)

catkin_package(
  CATKIN_DEPENDS exotica
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
