cmake_minimum_required(VERSION 2.8.3)
project(collision_scene_bullet)

add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)

set(bullet_FOUND 0)
pkg_check_modules(bullet REQUIRED bullet)

find_package(catkin REQUIRED COMPONENTS
  exotica
  geometric_shapes
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_scene_bullet
  CATKIN_DEPENDS exotica geometric_shapes
  DEPENDS bullet
)

include_directories(
  include ${bullet_INCLUDE_DIRS}
  ${bullet_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CollisionSceneBullet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${bullet_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
