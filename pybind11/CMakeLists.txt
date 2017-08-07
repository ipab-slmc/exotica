cmake_minimum_required(VERSION 2.8.12)
project(pybind11)

find_package(catkin REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(pybind)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

file(MAKE_DIRECTORY include)
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PYTHON_LIBRARIES}
  CATKIN_DEPENDS
  CFG_EXTRAS pybind.cmake
)

set(PYBIND11_HEADERS
  include/pybind11/attr.h
  include/pybind11/buffer_info.h
  include/pybind11/cast.h
  include/pybind11/chrono.h
  include/pybind11/class_support.h
  include/pybind11/common.h
  include/pybind11/complex.h
  include/pybind11/descr.h
  include/pybind11/eigen.h
  include/pybind11/embed.h
  include/pybind11/eval.h
  include/pybind11/functional.h
  include/pybind11/numpy.h
  include/pybind11/operators.h
  include/pybind11/options.h
  include/pybind11/pybind11.h
  include/pybind11/pytypes.h
  include/pybind11/stl.h
  include/pybind11/stl_bind.h
  include/pybind11/typeid.h
)

string(REPLACE "include/" "${CMAKE_CURRENT_SOURCE_DIR}/pybind11/include/"
       PYBIND11_HEADERS "${PYBIND11_HEADERS}")

# extract project version from source
file(STRINGS "${PYBIND11_INCLUDE_DIR}/pybind11/common.h" pybind11_version_defines
    REGEX "#define PYBIND11_VERSION_(MAJOR|MINOR|PATCH) ")
foreach(ver ${pybind11_version_defines})
 if (ver MATCHES "#define PYBIND11_VERSION_(MAJOR|MINOR|PATCH) +([^ ]+)$")
   set(PYBIND11_VERSION_${CMAKE_MATCH_1} "${CMAKE_MATCH_2}" CACHE INTERNAL "")
 endif()
endforeach()
set(${PROJECT_NAME}_VERSION ${PYBIND11_VERSION_MAJOR}.${PYBIND11_VERSION_MINOR}.${PYBIND11_VERSION_PATCH})
message(STATUS "pybind11 v${${PROJECT_NAME}_VERSION}")

install(FILES ${PYBIND11_HEADERS}
        DESTINATION include/pybind11)

set(PYBIND11_CMAKECONFIG_INSTALL_DIR "${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/" CACHE STRING "install path for pybind11Config.cmake")

install(FILES pybind11/tools/FindPythonLibsNew.cmake
              pybind11/tools/pybind11Tools.cmake
        DESTINATION ${PYBIND11_CMAKECONFIG_INSTALL_DIR})
