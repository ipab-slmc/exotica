cmake_minimum_required(VERSION 2.8.3)
project(collision_scene_fcl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  exotica
  geometric_shapes
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES collision_scene_fcl
  CATKIN_DEPENDS exotica geometric_shapes
)

# FCL 0.5.0 used in kinetic breaks the API.
# The ROS_KINETIC flag takes care of that in our code.
if ($ENV{ROS_DISTRO} MATCHES "(kinetic|lunar)")
    add_definitions(-DROS_KINETIC)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CollisionSceneFCL.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
