cmake_minimum_required(VERSION 2.8.3)
project(collision_scene_fcl_latest)

add_compile_options(-std=c++11)

# Find fcl_catkin
find_package(catkin REQUIRED COMPONENTS fcl_catkin)
set(fcl_catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
set(fcl_catkin_LIBRARIES ${catkin_LIBRARIES})

# Find other catkin packages
find_package(catkin REQUIRED COMPONENTS exotica geometric_shapes)
set(exotica_INCLUDES ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_scene_fcl_latest
  CATKIN_DEPENDS fcl_catkin geometric_shapes
)

include_directories(
  include
  ${fcl_catkin_INCLUDE_DIRS}
  ${fcl_catkin_INCLUDE_DIRS}/fcl_catkin
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CollisionSceneFCLLatest.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${fcl_catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
