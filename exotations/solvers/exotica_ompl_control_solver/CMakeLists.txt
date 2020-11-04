cmake_minimum_required(VERSION 3.0.2)
project(exotica_ompl_control_solver)

find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
)

AddInitializer(ompl_control_solver)
AddInitializer(control_rrt_solver)
AddInitializer(control_kpiece_solver)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${OMPL_LIBRARIES}
  CATKIN_DEPENDS exotica_core
  DEPENDS OMPL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/ompl_control_solver.cpp
  src/ompl_native_solvers.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

