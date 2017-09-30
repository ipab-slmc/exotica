cmake_minimum_required(VERSION 2.8.3)
project(collision_scene_fcl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  exotica
  geometric_shapes
)

# From Kinetic onwards, system installed FCL is used.
# No CMake config is provided either, we have to use pkgconfig.
if ($ENV{ROS_DISTRO} MATCHES "(kinetic|lunar)")
    find_package(PkgConfig)
    pkg_check_modules(FCL REQUIRED fcl)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES collision_scene_fcl
      CATKIN_DEPENDS exotica geometric_shapes
    )
    add_definitions(-DROS_KINETIC)
else()
    find_package(catkin REQUIRED COMPONENTS
      fcl
    )
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES collision_scene_fcl
      CATKIN_DEPENDS exotica geometric_shapes fcl
    )
endif()

include_directories(
  include
  ${FCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CollisionSceneFCL.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${FCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
