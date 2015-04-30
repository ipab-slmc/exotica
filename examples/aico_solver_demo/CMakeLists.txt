set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)

cmake_minimum_required(VERSION 2.8.3)
project(aico_solver_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  exotica
  aico
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp exotica aico sensor_msgs
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
add_executable(AICOexample src/AICOexample.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(AICOexample
  ${catkin_LIBRARIES}
  aico
  exotica
)
