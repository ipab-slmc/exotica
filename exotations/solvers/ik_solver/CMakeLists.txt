cmake_minimum_required(VERSION 2.8.3)
project(ik_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica
)


find_package(Boost REQUIRED COMPONENTS signals)

find_package(LAPACK REQUIRED)

AddInitializer(IKsolver)
GenInitializers()

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ik_solver
	CATKIN_DEPENDS exotica
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ik_solver
  src/ik_solver/IKSolver.cpp
)

target_link_libraries(ik_solver
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES}
)

add_dependencies(ik_solver ik_solver_initializers)

install(TARGETS ik_solver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
