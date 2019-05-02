cmake_minimum_required(VERSION 2.8.3)
project(exotica_collision_scene_fcl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  geometric_shapes
  roscpp
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core geometric_shapes roscpp
  DEPENDS LIBFCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/collision_scene_fcl.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBFCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
