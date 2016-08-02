cmake_minimum_required(VERSION 2.8.3)
project(aico)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  exotica
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)
find_package(LAPACK REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aico
  CATKIN_DEPENDS exotica
  DEPENDS system_lib
)

AddInitializer(AICOsolver)
GenInitializers()

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(aico
  src/AICOsolver.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(aico
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES}
)
add_dependencies(aico aico_initializers)

## Install
install(TARGETS aico
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
