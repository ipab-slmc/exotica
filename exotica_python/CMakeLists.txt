cmake_minimum_required(VERSION 2.8.3)
project(exotica_python)

# Python2 uses char* and Python3 const char* for arguments so explicit casting
# of string literals would require define flags. Alternatively, deactivate
# warnings for now
add_definitions(-Wno-write-strings)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  pybind11_catkin
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS exotica_core pybind11_catkin
)

include_directories(${catkin_INCLUDE_DIRS})
pybind_add_module(_pyexotica MODULE src/pyexotica.cpp)
target_link_libraries(_pyexotica PRIVATE ${catkin_LIBRARIES})
add_dependencies(_pyexotica ${catkin_EXPORTED_TARGETS})

catkin_python_setup()

install(TARGETS _pyexotica LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
