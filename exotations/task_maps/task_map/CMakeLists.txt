cmake_minimum_required(VERSION 2.8.12)
project(task_map)

find_package(catkin REQUIRED COMPONENTS
  exotica
  exotica_python
  geometry_msgs
)

AddInitializer(
  CoM
  IMesh
  EffPosition
  EffOrientation
  EffFrame
  FrameWithAxisAndDirection
  EffAxisAlignment
  EffVelocity
  JointLimit
  JointVelocityLimit
  Distance
  Point2Line
  Identity
  SphereCollision
  Sphere
  CollisionCheck
  Point2Plane
  QuasiStatic
  JointVelocityBackwardDifference
  JointAccelerationBackwardDifference
  JointJerkBackwardDifference
  LookAt
)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/CoM.cpp
    src/IMesh.cpp
    src/EffPosition.cpp
    src/EffOrientation.cpp
    src/EffFrame.cpp
    src/EffAxisAlignment.cpp
    src/EffVelocity.cpp
    src/JointLimit.cpp
    src/JointVelocityLimit.cpp
    src/Distance.cpp
    src/Point2Line.cpp
    src/Point2Plane.cpp
    src/Identity.cpp
    src/SphereCollision.cpp
    src/CollisionCheck.cpp
    src/QuasiStatic.cpp
    src/JointVelocityBackwardDifference.cpp
    src/JointAccelerationBackwardDifference.cpp
    src/JointJerkBackwardDifference.cpp
    src/LookAt.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

pybind_add_module(${PROJECT_NAME}_py MODULE src/task_map_py.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_py ${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME}_py LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_maps tests/test_maps.cpp)
  target_link_libraries(test_maps ${catkin_LIBRARIES})
  add_dependencies(test_maps ${catkin_EXPORTED_TARGETS})
endif()
