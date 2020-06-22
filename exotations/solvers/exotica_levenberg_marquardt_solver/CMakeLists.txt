cmake_minimum_required(VERSION 3.0.2)
project(exotica_levenberg_marquardt_solver)

find_package(catkin REQUIRED COMPONENTS exotica_core)

find_package(Eigen3 REQUIRED)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "For best performance, the Levenberg-Marquardt solver requires at least Eigen version 3.3. Using an alternative linear solver.")
endif()

AddInitializer(levenberg_marquardt_solver)
GenInitializers()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS exotica_core
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/levenberg_marquardt_solver.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
