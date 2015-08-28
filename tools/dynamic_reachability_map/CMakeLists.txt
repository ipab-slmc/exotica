cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reachability_map)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  message_generation
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES DRM.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  exotica
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_reachability_map
  CATKIN_DEPENDS exotica roscpp actionlib_msgs message_generation
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(dynamic_reachability_map
  src/dynamic_reachability_map.cpp
)
target_link_libraries(dynamic_reachability_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(DRMActionNode src/DRMActionNode.cpp src/dynamic_reachability_map.cpp)
target_link_libraries(DRMActionNode
  ${catkin_LIBRARIES}
)

add_executable(DRMSpaceSampler src/drm_space_sampler.cpp src/dynamic_reachability_map.cpp)
target_link_libraries(DRMSpaceSampler
  ${catkin_LIBRARIES}
)


