set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)

cmake_minimum_required(VERSION 2.8.3)
project(ik_solver_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  exotica_distro
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp exotica_distro sensor_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp executable
add_executable(IKexample src/node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(IKexample
  ${catkin_LIBRARIES}
)
