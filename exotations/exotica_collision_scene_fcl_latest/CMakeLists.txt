cmake_minimum_required(VERSION 2.8.3)
project(exotica_collision_scene_fcl_latest)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS exotica_core fcl_catkin geometric_shapes)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core fcl_catkin geometric_shapes
)

include_directories(
  include
  ${fcl_catkin_INCLUDE_DIRS}/fcl_catkin
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/collision_scene_fcl_latest.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
