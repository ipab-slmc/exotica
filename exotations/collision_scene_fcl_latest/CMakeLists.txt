cmake_minimum_required(VERSION 2.8.3)
project(collision_scene_fcl_latest)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  exotica
  geometric_shapes
)

find_package(fcl 0.6.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_scene_fcl_latest
  CATKIN_DEPENDS exotica geometric_shapes
)

include_directories(
  include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CollisionSceneFCLLatest.cpp
)

target_link_libraries(${PROJECT_NAME}
  fcl
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
