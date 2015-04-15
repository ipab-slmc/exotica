cmake_minimum_required(VERSION 2.8.3)
project(task_definition)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
add_definitions(-DEXOTICA_DEBUG_MODE) 

find_package(catkin REQUIRED COMPONENTS
  exotica
)

##find_package(EIGEN REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica
  DEPENDS
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
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/TaskSqrError.cpp
                            src/TaskTerminationCriterion.cpp
           )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
