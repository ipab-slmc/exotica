cmake_minimum_required(VERSION 2.8.3)
project(exotica_examples)

find_package(catkin REQUIRED COMPONENTS
  exotica
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(exotica_core src/core.cpp)

target_link_libraries(exotica_core
  ${catkin_LIBRARIES}
)
