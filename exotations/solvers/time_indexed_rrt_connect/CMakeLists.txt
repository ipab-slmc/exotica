cmake_minimum_required(VERSION 2.8.3)
project(time_indexed_rrt_connect)

find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  exotica
)

catkin_package(
  INCLUDE_DIRS include ${OMPL_INCLUDE_DIRS}
  LIBRARIES time_indexed_rrt_connect ${OMPL_LIBRARIES}
  CATKIN_DEPENDS exotica
)

AddInitializer(TimeIndexedRRTConnect)
GenInitializers()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  SYSTEM
)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_library(time_indexed_rrt_connect
  src/TimeIndexedRRTConnect.cpp
)
target_link_libraries(time_indexed_rrt_connect
  ${catkin_LIBRARIES} ${OMPL_LIBRARIES}
)

add_dependencies(time_indexed_rrt_connect time_indexed_rrt_connect_initializers)

install(TARGETS time_indexed_rrt_connect
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

