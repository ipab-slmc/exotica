cmake_minimum_required(VERSION 2.8.3)
project(exotica_levenberg_marquardt_solver)

find_package(catkin REQUIRED COMPONENTS exotica)

find_package(Eigen3 REQUIRED)

AddInitializer(LevenbergMarquardtSolver)
GenInitializers()

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS exotica
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/LevenbergMarquardtSolver.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
