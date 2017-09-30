cmake_minimum_required(VERSION 2.8.12)
project(pybind11_catkin)

find_package(catkin REQUIRED)

include(ExternalProject)

set(PYBIND_SRC ${CMAKE_BINARY_DIR}/pybind11_src)

ExternalProject_Add(
  pybind11
  GIT_REPOSITORY "https://github.com/pybind/pybind11.git"
  GIT_TAG "391c75447da13ef44cc2e119e005c0aeee442862"

  UPDATE_COMMAND ""
  PATCH_COMMAND ""

  SOURCE_DIR "${PYBIND_SRC}"
  CMAKE_ARGS -DPYBIND11_TEST=No -DPYBIND11_INSTALL=Yes -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}

  TEST_COMMAND ""
)

ExternalProject_Add_Step(
  pybind11 MOVEToShared
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CATKIN_DEVEL_PREFIX}/share/cmake/pybind11 ${CATKIN_DEVEL_PREFIX}/share/pybind11/cmake
  COMMAND ${CMAKE_COMMAND} -E copy ${CATKIN_DEVEL_PREFIX}/share/cmake/pybind11/pybind11Tools.cmake ${CATKIN_DEVEL_PREFIX}/share/pybind11_catkin/cmake
  COMMAND ${CMAKE_COMMAND} -E copy ${CATKIN_DEVEL_PREFIX}/share/cmake/pybind11/FindPythonLibsNew.cmake ${CATKIN_DEVEL_PREFIX}/share/pybind11_catkin/cmake
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${CATKIN_DEVEL_PREFIX}/share/cmake/
  DEPENDEES install
)


file(MAKE_DIRECTORY include)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PYTHON_LIBRARIES}
  CATKIN_DEPENDS
  CFG_EXTRAS pybind.cmake
  DEPENDS pybind11
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/pybind11/cmake/ DESTINATION share/pybind11/cmake)
install(FILES ${CATKIN_DEVEL_PREFIX}/share/pybind11/cmake/pybind11Tools.cmake ${CATKIN_DEVEL_PREFIX}/share/pybind11/cmake/FindPythonLibsNew.cmake DESTINATION share/pybind11_catkin/cmake)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/pybind11/ DESTINATION include/pybind11)

