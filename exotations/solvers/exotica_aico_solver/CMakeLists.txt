cmake_minimum_required(VERSION 2.8.12)
project(exotica_aico_solver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  exotica
)

## System dependencies are found with CMake's conventions
find_package(LAPACK REQUIRED)

AddInitializer(
  ApproximateInferenceSolver
  AICOSolver
  BayesianIKSolver
)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica
  DEPENDS LAPACK
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/math_operations.cpp
  src/aico_solver.cpp
  src/bayesian_ik_solver.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LAPACK_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_initializers
  ${catkin_EXPORTED_TARGETS}
)

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
