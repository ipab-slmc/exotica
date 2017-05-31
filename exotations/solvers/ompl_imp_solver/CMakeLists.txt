cmake_minimum_required(VERSION 2.8.3)
project(ompl_imp_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica
  ompl_solver
  pluginlib
)

find_package(Boost REQUIRED COMPONENTS system)

# Check ROS version
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (${ROS_VERSION} MATCHES "(indigo|jade)")
  add_definitions(-DROS_INDIGO)
else (${ROS_VERSION} MATCHES "(kinetic|lunar")
  add_definitions(-DROS_KINETIC)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_imp_solver
  CATKIN_DEPENDS exotica ompl_solver pluginlib
)

AddInitializer(OMPLImplementation)
GenInitializers()

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
add_dependencies(ompl_imp_solver ompl_imp_solver_initializers)

install(TARGETS ompl_imp_solver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
