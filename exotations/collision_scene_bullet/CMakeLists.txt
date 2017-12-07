cmake_minimum_required(VERSION 2.8.12)
project(collision_scene_bullet)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  exotica
  geometric_shapes
)

find_package(Bullet REQUIRED COMPONENTS BulletCollision)
message(WARNING ${BULLET_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_scene_bullet
  CATKIN_DEPENDS geometric_shapes
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/CollisionSceneBullet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${BULLET_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
