cmake_minimum_required(VERSION 3.0.2)
project(exotica_pendulum_dynamics_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  roscpp
)

AddInitializer(pendulum_dynamics_solver)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/pendulum_dynamics_solver.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Ignore Eigen::Tensor warnings
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-ignored-attributes)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
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

# if(CATKIN_ENABLE_TESTING)
  # catkin_add_nosetests(test/test_pendulum_diff.py)
# endif()
