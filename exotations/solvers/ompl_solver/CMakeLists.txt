cmake_minimum_required(VERSION 2.8.3)
project(ompl_solver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
add_definitions(-DEXOTICA_DEBUG_MODE)
find_package(OMPL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  roslib
  task_definition
  task_map
  ik_solver
  pluginlib
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ompl_solver
  CATKIN_DEPENDS exotica roscpp roslib task_definition task_map ik_solver pluginlib
  DEPENDS system_lib ompl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  SYSTEM
)

add_library(ompl_solver
   src/OMPLsolver.cpp
   src/OMPLProblem.cpp
   src/OMPLBaseSolver.cpp
   src/OMPLBaseStateSpace.cpp
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

target_link_libraries(ompl_solver
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${OMPL_LIBRARIES}
)

add_library(ompl_imp_solver
   src/OMPLImpSolver.cpp
   src/OMPLRNStateSpace.cpp
   src/OMPLSE3RNStateSpace.cpp
)
target_link_libraries(ompl_imp_solver
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${OMPL_LIBRARIES}
)

install(TARGETS ompl_solver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
