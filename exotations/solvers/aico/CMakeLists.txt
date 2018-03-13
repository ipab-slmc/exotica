cmake_minimum_required(VERSION 2.8.12)
project(aico)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  exotica
)

## System dependencies are found with CMake's conventions
find_package(LAPACK REQUIRED)

AddInitializer(
  AICOsolver
  BayesianIK
)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aico
  CATKIN_DEPENDS exotica
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(aico
  src/MathOperations.cpp
  src/AICOsolver.cpp
  src/BayesianIK.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(aico
  ${catkin_LIBRARIES} ${LAPACK_LIBRARIES}
)
add_dependencies(aico aico_initializers)

## Install
install(TARGETS aico
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
