cmake_minimum_required(VERSION 2.8.3)
project(testing_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
#add_definitions(-DEXOTICA_DEBUG_MODE)


find_package(catkin REQUIRED COMPONENTS
  kinematica
  exotica
  task_map
  task_definition
  roscpp
  rostest
  roslib
)

## System dependencies are found with CMake's conventions
find_package(orocos_kdl REQUIRED)
##find_package(EIGEN REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS kinematica roscpp roslib exotica task_map task_definition
 DEPENDS system_lib orocos_kdl eigen
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_rostest_gtest(kinematica_test test/rostest/kinematica_test.test test/main/TestKinematica.cpp test/cpp/Kinematica/ComplexityTest.cpp test/cpp/Kinematica/PrismaticTest.cpp)
target_link_libraries(kinematica_test ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES})

add_rostest_gtest(exotica_test test/rostest/exotica_test.test test/main/TestExotica.cpp
                                                              test/cpp/Exotica/CoreTest.cpp
                                                              #test/cpp/Exotica/TaskMapTest.cpp
                                                              #test/cpp/Exotica/TaskDefinitionTest.cpp
                                                              #test/cpp/Exotica/PlanningProblemTest.cpp
                 )
                 
target_link_libraries(exotica_test ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_LIBRARIES})
