cmake_minimum_required(VERSION 3.22)
project(exotica_examples)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(exotica_core REQUIRED)
find_package(exotica_aico_solver REQUIRED)
find_package(exotica_core_task_maps REQUIRED)
find_package(exotica_ik_solver REQUIRED)
find_package(rclcpp REQUIRED)

# Helper to add an example executable
function(add_example target src)
  add_executable(${target} ${src})
  ament_target_dependencies(${target} PUBLIC exotica_core rclcpp)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_example(example_cpp_core          src/core.cpp)
add_example(example_cpp_ik_minimal    src/ik_minimal.cpp)
add_example(example_cpp_init_generic  src/generic.cpp)
add_example(example_cpp_init_xml      src/xml.cpp)
add_example(example_cpp_planner       src/planner.cpp)
add_example(example_cpp_xml_load_and_solve src/xml_load_and_solve.cpp)

ament_python_install_package(exotica_examples_py
  PACKAGE_DIR src/exotica_examples_py)

install(DIRECTORY launch/    DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY resources/ DESTINATION share/${PROJECT_NAME}/resources)
install(DIRECTORY scripts/   DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  install(DIRECTORY test/resources/ DESTINATION share/${PROJECT_NAME}/test/resources)

  ament_add_gtest(test_initializers test/test_initializers.cpp)
  ament_target_dependencies(test_initializers exotica_core rclcpp)

  ament_add_gtest(test_problems test/test_problems.cpp)
  ament_target_dependencies(test_problems exotica_core)
endif()

ament_package()
