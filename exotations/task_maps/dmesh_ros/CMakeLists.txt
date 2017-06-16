cmake_minimum_required(VERSION 2.8.3)
project(dmesh_ros)

find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  ik_solver
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

AddInitializer(DMeshROS)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmesh_ros
  CATKIN_DEPENDS exotica roscpp ik_solver visualization_msgs
)

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
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
