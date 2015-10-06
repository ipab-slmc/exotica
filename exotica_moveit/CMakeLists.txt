cmake_minimum_required(VERSION 2.8.3)
project(exotica_moveit)

## Set Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
add_definitions(-DEXOTICA_DEBUG_MODE)  #Controls whether to compile with certain debug checks included (-DEXOTICA_DEBUG_MODE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  exotica_distro
  dynamic_reconfigure
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
  tf
  eigen_conversions
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)


find_package(Boost REQUIRED system filesystem date_time thread serialization)

add_action_files(
  DIRECTORY action
  FILES ExoticaPlanning.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
  exotica
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES exotica_moveit
  CATKIN_DEPENDS exotica_distro dynamic_reconfigure eigen_conversions moveit_core moveit_ros_planning_interface pluginlib roscpp tf rospy std_msgs message_generation actionlib_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include)
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(moveit_exotica_planner_plugin src/EXOTicaPlannerManager.cpp)
target_link_libraries(moveit_exotica_planner_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(${PROJECT_NAME} src/EXOTicaPlannerService.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ExoticaPlannerServiceNode src/EXOTicaPlannerServiceNode.cpp src/EXOTicaPlannerService.cpp)
target_link_libraries(ExoticaPlannerServiceNode ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ExoticaPlannerServiceNode exotica_moveit_gencpp)

#############
## Install ##
#############

install(FILES exotica_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(TARGETS moveit_exotica_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

