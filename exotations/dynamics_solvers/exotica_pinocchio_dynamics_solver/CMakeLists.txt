cmake_minimum_required(VERSION 3.0.2)
project(exotica_pinocchio_dynamics_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  roscpp
)

find_package(pinocchio REQUIRED)

AddInitializer(
  pinocchio_dynamics_solver
  pinocchio_gravity_compensation_dynamics_solver
)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core
  DEPENDS pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pinocchio_dynamics_solver.cpp
  src/pinocchio_dynamics_solver_derivatives.cpp
  src/pinocchio_gravity_compensation_dynamics_solver.cpp
  src/pinocchio_gravity_compensation_dynamics_solver_derivatives.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated -Wno-variadic-macros -Wno-deprecated-declarations -Wno-comment -Wno-ignored-attributes)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  exotica_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
