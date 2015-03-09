cmake_minimum_required(VERSION 2.8.3)
project(dmesh_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  ik_solver
  task_definition
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)




catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmesh_ros
  CATKIN_DEPENDS exotica roscpp ik_solver task_definition  visualization_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/dmesh_ros.cpp
  src/GraphManager.cpp
  src/MeshGraph.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
## Declare a cpp executable
# add_executable(dmesh_ros_node src/dmesh_ros_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(dmesh_ros_node dmesh_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(dmesh_ros_node
#   ${catkin_LIBRARIES}
# )
