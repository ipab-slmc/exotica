# EXOTica [![Build Status](https://travis-ci.org/ipab-slmc/exotica.svg?branch=master)](https://travis-ci.org/ipab-slmc/exotica) [![Analytics](https://ga-beacon.appspot.com/UA-72496975-1/openhumanoids/exotica/?pixel)](https://github.com/igrigorik/ga-beacon)

[Documentation](http://ipab-slmc.github.io/exotica/) - [C++ Doxygen](http://ipab-slmc.github.io/exotica/doxygen_cpp/) - [Python Documentation](https://ipab-slmc.github.io/exotica/Python-API.html)

The EXOTica library is a generic Optimisation Toolset for Robotics platforms, written in C++ with bindings for Python. Its motivation is to provide a more streamlined process for developing algorithms for tasks such as Inverse Kinematics and Trajectory Optimisation. Its design advocates:

 * **Modularity:** The library is developed in a modular manner making use of C++’s object-oriented features (such as polymorphism). This allows users to define their own components and ’plug them into’ the existing framework. Thus, an engineer does not need to implement a whole system whenever he needs to change a component, but rather can re-implement the specific functionality and as long as he follows certain guidelines, retain the use of the other modules.
 * **Extensibility:** The library is also heavily extensible, mainly thanks to the modular design. In addition, the library makes very minimal prior assumptions about the form of the problem so that it can be as generic as possible.
 * **Integration with ROS:** The library is designed to be fully integrated with ROS allowing to set up, configuration, consuming data from ros topics, and publishing debug display using ROS tools.

The library itself consists of two major specifications, both of which are abstract classes. The first is the *Motion Solver* which defines the way optimisation should proceed: current implementation include AICO, Jacobian pseudo-inverse IK, and a range of sampling-based solvers from the [OMPL](http://ompl.kavrakilab.org/) library. The other is the *Task Definition* which describes the task itself by providing two necessary functions to compute the forward map from Configuration space (say joint angles in IK) to Task space (say end-effector positions in IK). The tasks themselves can describe a complete trajectory. Using the library then involves passing in an initial state and requesting a solution to the problem, which may consist of a single configuration or complete trajectory.

# Prerequisites
* Ubuntu 16.04 with ROS Kinetic (primary and tested with continuous integration). Ubuntu 14.04/ROS Indigo and Ubuntu 18.04/ROS Melodic work (more details below) but are not officially supported or covered by continuous integration.
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) (```catkin_make``` is no longer supported)
* [rosdep](http://wiki.ros.org/rosdep)
* [ROS](http://wiki.ros.org/Installation) (```ros-[release]-desktop``` is recommended but more minimal versions work in conjunction with rosdep)
* ``moveit-ros-planning-interface``

## ROS Indigo (Ubuntu 14.04)
We retired support for ROS Indigo in September 2018. For using 14.04, a number of manual changes are required:
* For Eigen, we recommend system-installing 3.2.10 or newer, for which we provide a [Debian](http://terminator.robots.inf.ed.ac.uk/apt/libeigen3-dev.deb).
* gcc >[4.9](https://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu) -- the 14.04 system-installed 4.8.4 won't work. On Ubuntu 16.04, you can use the system-provided gcc/g++.
* For compiling ``fcl_catkin``, you need to add a PPA for ``libccd-dev``.

## ROS Melodic (Ubuntu 18.04)
These workarounds are currently required on ROS Melodic:
* Please clone ``pybind11_catkin`` into your source directory from [here](https://github.com/ipab-slmc/pybind11_catkin).

# Installation

1. [Create a catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace) or use an existing workspace. [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) is the preferred build system.
2. Clone this repository into the ```src/``` subdirectory of the workspace (any subdirectory below ```src/``` will do): ``git clone git@github.com:ipab-slmc/exotica.git --recursive``.
3. ```cd``` into the the cloned directory.
4. Install dependencies
  1. If running rosdep for the first time start by running:
     ```sudo rosdep init```
  2. ```rosdep update ; rosdep install --from-paths ./ -iy ```
5. Compile the code ```catkin build -s```.
6. Source the config file (ideally inside ```~/.bashrc```): ```source path_to_workspace/devel/setup.bash```. You may have to source the config file from your installspace if your workspace is configured for installation.

# Demos
Have a look at ```examples/exotica_examples```.
If you have sourced the workspace correctly you should be able to run any of the demos:

```
roslaunch exotica_examples CppCore.launch
roslaunch exotica_examples CppPlanAICO.launch
roslaunch exotica_examples CppPlanOMPL.launch
roslaunch exotica_examples CppInitGeneric.launch
roslaunch exotica_examples CppInitManual.launch
roslaunch exotica_examples CppInitXML.launch
roslaunch exotica_examples PythonAttachDemo.launch
roslaunch exotica_examples PythonCollisionDistance.launch
```

# Publications

We have published a [Springer book chapter](https://link.springer.com/chapter/10.1007/978-3-319-91590-6_7) outlining the concept and ideas behind EXOTica and recommend it to new users for getting started in addition to the tutorials and documentation.

> Ivan V., Yang Y., Merkt W., Camilleri M.P., Vijayakumar S. (2019) EXOTica: An Extensible Optimization Toolset for Prototyping and Benchmarking Motion Planning and Control. In: Koubaa A. (eds) Robot Operating System (ROS). Studies in Computational Intelligence, vol 778. Springer, Cham

If you use EXOTica for academic work, please cite consider citing the relevant book chapter, a preprint of which is available [here](https://vladimirivan.files.wordpress.com/2018/03/exoticarosbook.pdf):
     
    @Inbook{exotica,
        author="Ivan, Vladimir and Yang, Yiming and Merkt, Wolfgang and Camilleri, Michael P. and Vijayakumar, Sethu",
        editor="Koubaa, Anis",
        title="EXOTica: An Extensible Optimization Toolset for Prototyping and Benchmarking Motion Planning and Control",
        bookTitle="Robot Operating System (ROS): The Complete Reference (Volume 3)",
        year="2019",
        publisher="Springer International Publishing",
        address="Cham",
        pages="211--240",
        isbn="978-3-319-91590-6",
        doi="10.1007/978-3-319-91590-6_7",
        url="https://doi.org/10.1007/978-3-319-91590-6_7"
    }
