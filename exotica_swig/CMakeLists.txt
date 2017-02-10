cmake_minimum_required(VERSION 2.8.3)
project(exotica_swig)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  exotica
  swigmake
)

find_package(Boost REQUIRED COMPONENTS signals)
find_package(SWIG REQUIRED)
find_package(Eigen3 3.2.10 REQUIRED)
find_package(NumPy REQUIRED)
mex_setup(REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exotica_swig
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
) 

set_property(SOURCE swig/exotica.i swig/exotica_python.i swig/exotica_matlab.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE swig/exotica.i swig/exotica_python.i swig/exotica_matlab.i PROPERTY SWIG_MODULE_NAME exotica)
add_swig_python_module(exopy swig/exotica_python.i CPLUSPLUS LINK_LIBRARIES ${catkin_LIBRARIES})
add_swig_matlab_module(exomat swig/exotica_matlab.i CPLUSPLUS LINK_LIBRARIES ${catkin_LIBRARIES})

