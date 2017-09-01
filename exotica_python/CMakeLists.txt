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

pybind_add_module(_pyexotica MODULE src/Exotica.cpp)

catkin_python_setup()

install(TARGETS _pyexotica LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
catkin_install_python(PROGRAMS
    scripts/example.py
    scripts/example_aico.py
    scripts/example_ompl.py
    scripts/example_ik.py
    scripts/example_ik_noros.py
    scripts/example_aico_noros.py
    scripts/example_ompl_noros.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING)
    catkin_install_python(PROGRAMS tests/core.py tests/runtest.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif(CATKIN_ENABLE_TESTING)
