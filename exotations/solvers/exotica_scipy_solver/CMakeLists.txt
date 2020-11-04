cmake_minimum_required(VERSION 3.0.2)
project(exotica_scipy_solver)

find_package(catkin REQUIRED COMPONENTS
    exotica_core
)
catkin_python_setup()
catkin_package()
