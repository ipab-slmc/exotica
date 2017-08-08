cmake_minimum_required(VERSION 2.8.3)
project(exotica_python)

find_package(catkin REQUIRED COMPONENTS
  exotica
  pybind11_catkin
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS exotica pybind11_catkin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

pybind_add_module(pyexotica_impl MODULE src/Exotica.cpp)

catkin_python_setup()

install(TARGETS pyexotica_impl LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
catkin_install_python(PROGRAMS scripts/example_aico.py scripts/example_ompl.py scripts/example_ik.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
