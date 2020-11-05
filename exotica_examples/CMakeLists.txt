cmake_minimum_required(VERSION 3.0.2)
project(exotica_examples)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  exotica_python
  exotica_aico_solver
  exotica_core_task_maps
  exotica_ik_solver
  sensor_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(example_cpp_core src/core.cpp)
target_link_libraries(example_cpp_core ${catkin_LIBRARIES})
add_dependencies(example_cpp_core ${catkin_EXPORTED_TARGETS})

add_executable(example_cpp_init_generic src/generic.cpp)
target_link_libraries(example_cpp_init_generic ${catkin_LIBRARIES})
add_dependencies(example_cpp_init_generic ${catkin_EXPORTED_TARGETS})

add_executable(example_cpp_init_xml src/xml.cpp)
target_link_libraries(example_cpp_init_xml ${catkin_LIBRARIES})
add_dependencies(example_cpp_init_xml ${catkin_EXPORTED_TARGETS})

add_executable(example_cpp_xml_load_and_solve src/xml_load_and_solve.cpp)
target_link_libraries(example_cpp_xml_load_and_solve ${catkin_LIBRARIES})
add_dependencies(example_cpp_xml_load_and_solve ${catkin_EXPORTED_TARGETS})

add_executable(example_cpp_planner src/planner.cpp)
target_link_libraries(example_cpp_planner ${catkin_LIBRARIES})
add_dependencies(example_cpp_planner ${catkin_EXPORTED_TARGETS})

add_executable(example_cpp_ik_minimal src/ik_minimal.cpp)
target_link_libraries(example_cpp_ik_minimal ${catkin_LIBRARIES})
add_dependencies(example_cpp_ik_minimal ${catkin_EXPORTED_TARGETS})

install(TARGETS
  example_cpp_init_generic
  example_cpp_init_xml
  example_cpp_xml_load_and_solve
  example_cpp_planner
  example_cpp_core
  example_cpp_ik_minimal
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY resources/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)
FILE(GLOB python_examples "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
catkin_install_python(PROGRAMS "${python_examples}" DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # Find again, this time including rostest
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_initializers test/test_initializers.cpp)
  target_link_libraries(test_initializers ${catkin_LIBRARIES})
  add_dependencies(test_initializers ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(test_problems test/test_problems.cpp)
  target_link_libraries(test_problems ${catkin_LIBRARIES})
  add_dependencies(test_problems ${catkin_EXPORTED_TARGETS})

  add_rostest(test/python_tests.launch)

  catkin_add_nosetests(test/test_ompl_solver_bounds.py)
  catkin_add_nosetests(test/test_dynamics_solvers.py)
  catkin_add_nosetests(test/test_dynamic_time_indexed_shooting_problem.py)
endif()
