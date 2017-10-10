cmake_minimum_required(VERSION 2.8.3)
project(ompl_solver)

find_package(OMPL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  exotica
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_solver
  CATKIN_DEPENDS exotica ompl
)

AddInitializer(OMPLsolver RRTConnect)
GenInitializers()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  SYSTEM
)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_library(ompl_solver
  src/ompl_solver/ompl_exo.cpp
  src/ompl_solver/ompl_solver.cpp
  src/ompl_solver/RRTConnect.cpp
)
target_link_libraries(ompl_solver
  ${catkin_LIBRARIES} ${OMPL_LIBRARIES}
)

add_dependencies(ompl_solver ompl_solver_initializers)

install(TARGETS ompl_solver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

