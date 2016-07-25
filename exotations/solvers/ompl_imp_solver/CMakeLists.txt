cmake_minimum_required(VERSION 2.8.3)
project(ompl_imp_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica_distro
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_imp_solver
  CATKIN_DEPENDS exotica_distro
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ompl_imp_solver
   src/${PROJECT_NAME}/OMPLImpSolver.cpp
   src/${PROJECT_NAME}/OMPLBaseStateSpace.cpp
   src/${PROJECT_NAME}/OMPLRNStateSpace.cpp
   src/${PROJECT_NAME}/OMPLSE3RNStateSpace.cpp
)
target_link_libraries(ompl_imp_solver
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${OMPL_LIBRARIES}
)
