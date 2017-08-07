cmake_minimum_required(VERSION 2.8.3)
project(ompl_solver)

add_definitions(-DEXOTICA_DEBUG_MODE)
find_package(OMPL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  exotica
  pluginlib
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ompl_base_solver
  CATKIN_DEPENDS exotica pluginlib ompl
)

AddInitializer(OMPLsolver)
GenInitializers()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  SYSTEM
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

add_library(ompl_base_solver
   src/OMPLBaseSolver.cpp
)

target_link_libraries(ompl_base_solver
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${OMPL_LIBRARIES}
)

install(TARGETS ompl_base_solver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

add_library(ompl_solver
   src/OMPLsolver.cpp
)

target_link_libraries(ompl_solver
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${OMPL_LIBRARIES} ompl_base_solver
)
add_dependencies(ompl_solver ompl_solver_initializers)

install(TARGETS ompl_solver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
