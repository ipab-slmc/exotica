cmake_minimum_required(VERSION 3.0.2)
project(exotica_collision_scene_fcl_latest)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS exotica_core geometric_shapes)

# FCL 0.6.x has been released into Noetic as FCL. We can thus use the upstream FCL version.
# For previous ROS releases, we used our own bleeding-edge catkin wrapper fcl_catkin.
# The following switches between the two variants
set(FCL_INCLUDE_DIRS "")
set(FCL_LIBRARIES "")
set(FCL_DEPENDENCY "")
set(FCL_CATKIN_DEPENDENCY "")
if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
  message(STATUS "Noetic - using ros-noetic-fcl")
  find_package(fcl REQUIRED)
  set(FCL_DEPENDENCY "FCL")
elseif(DEFINED ENV{ROS_DISTRO})
  message(STATUS "Non-Noetic ROS distribution ($ENV{ROS_DISTRO}) - using ros-noetic-fcl-catkin")
  find_package(fcl_catkin REQUIRED)
  set(FCL_INCLUDE_DIRS ${fcl_catkin_INCLUDE_DIRS}/fcl_catkin)
  set(FCL_LIBRARIES ${fcl_catkin_LIBRARIES})
  set(FCL_CATKIN_DEPENDENCY "fcl_catkin")
else()
  message(FATAL_ERROR "Unknown ROS_DISTRO - cannot select FCL dependency.")
endif()

AddInitializer(collision_scene_fcl_latest)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core ${FCL_CATKIN_DEPENDENCY} geometric_shapes
  DEPENDS ${FCL_DEPENDENCY}
)

include_directories(
  include
  ${FCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/collision_scene_fcl_latest.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${FCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
