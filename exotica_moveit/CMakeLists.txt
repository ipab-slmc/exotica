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
  aico
  ik_solver
  dynamic_reconfigure
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
  tf
  task_definition
  task_map 
  eigen_conversions
  rospy
  std_msgs
  message_generation
  exotica
  ompl_solver
)


find_package(Boost REQUIRED system filesystem date_time thread serialization)

#generate_dynamic_reconfigure_options("cfg/EXOTicaDynamicReconfigure.cfg")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
add_service_files(
  FILES
  ExoticaPlanning.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
  exotica
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES exotica_moveit
  CATKIN_DEPENDS aico ik_solver dynamic_reconfigure eigen_conversions moveit_core moveit_ros_planning_interface pluginlib roscpp tf task_definition task_map rospy std_msgs message_generation exotica ompl_solver
  DEPENDS system_lib
)

# planning_interface planning_scene_monitor

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(moveit_exotica_planner_plugin src/EXOTicaPlannerManager.cpp)
target_link_libraries(moveit_exotica_planner_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ExoticaPlanningServer src/EXOTicaPlannerServiceNode.cpp)
target_link_libraries(ExoticaPlanningServer ${catkin_LIBRARIES})
add_dependencies(ExoticaPlanningServer exotica_moveit_gencpp)

#############
## Install ##
#############

install(FILES exotica_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(TARGETS moveit_exotica_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

