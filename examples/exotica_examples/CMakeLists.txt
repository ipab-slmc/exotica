cmake_minimum_required(VERSION 2.8.3)
project(exotica_examples)

find_package(catkin REQUIRED COMPONENTS
  exotica
  exotica_ik_solver
  sensor_msgs
  task_map
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ExoticaCore src/core.cpp)
target_link_libraries(ExoticaCore ${catkin_LIBRARIES})
add_dependencies(ExoticaCore ${catkin_EXPORTED_TARGETS})

add_executable(ManualInitialization src/manual.cpp)
target_link_libraries(ManualInitialization ${catkin_LIBRARIES})
add_dependencies(ManualInitialization ${catkin_EXPORTED_TARGETS})

add_executable(GenericInitialization src/generic.cpp)
target_link_libraries(GenericInitialization ${catkin_LIBRARIES})
add_dependencies(GenericInitialization ${catkin_EXPORTED_TARGETS})

add_executable(XMLInitialization src/xml.cpp)
target_link_libraries(XMLInitialization ${catkin_LIBRARIES})
add_dependencies(XMLInitialization ${catkin_EXPORTED_TARGETS})

add_executable(PlannerDemo src/planner.cpp)
target_link_libraries(PlannerDemo ${catkin_LIBRARIES})
add_dependencies(PlannerDemo ${catkin_EXPORTED_TARGETS})

add_executable(IK src/ik_minimal.cpp)
target_link_libraries(IK ${catkin_LIBRARIES})
add_dependencies(IK ${catkin_EXPORTED_TARGETS})

install(TARGETS
  ManualInitialization
  GenericInitialization
  XMLInitialization
  PlannerDemo
  ExoticaCore
  IK
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY resources/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)
FILE(GLOB python_examples "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
catkin_install_python(PROGRAMS ${python_examples} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # Find again, this time including rostest
  find_package(catkin REQUIRED COMPONENTS
    exotica
    sensor_msgs
    exotica_ik_solver
    task_map
    rostest
  )

  catkin_add_gtest(test_initializers test/test_initializers.cpp)
  target_link_libraries(test_initializers ${catkin_LIBRARIES})
  add_dependencies(test_initializers ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(test_problems test/test_problems.cpp)
  target_link_libraries(test_problems ${catkin_LIBRARIES})
  add_dependencies(test_problems ${catkin_EXPORTED_TARGETS})

  add_rostest(test/python_tests.launch)

  catkin_add_nosetests(test/run_tests.py)
endif()
