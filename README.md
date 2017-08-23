# EXOTica [![Build Status](https://travis-ci.org/openhumanoids/exotica.svg?branch=master)](https://travis-ci.org/openhumanoids/exotica) [![Analytics](https://ga-beacon.appspot.com/UA-72496975-1/openhumanoids/exotica/?pixel)](https://github.com/igrigorik/ga-beacon)

The EXOTica library is a generic Optimisation Toolset for Robotics platforms, written in C++. Its motivation is to provide a more streamlined process for developing algorithms for such tasks as Inverse-Kinematics and Trajectory Optimisation. Its design advocates:

 * Modularity: The library is developed in a modular manner making use of C++’s object-oriented features (such as polymorphism). This allows users to define their own components and ’plug them into’ the existing framework. The end-effect is that an engineer need not implement a whole system whenever he needs to change a component, but rather can re-implement the specific functionality and as long as he follows certain guidelines, retain the use of the other modules.
 * Extensibility: The library is also heavily extensible, mainly thanks to the modular design. In addition, the library makes very minimal prior assumptions about the form of the problem so that it can be as generic as possible.
 * Integration with ROS: The library is designed to be fully integrated with ROS allowing to set up, configuration, consuming data from ros topics, and publishing debug display using ros tools.


The library itself consists of two major specifications, both of which are abstract classes. The first is the Problem Solver which defines the way optimisation should proceed: current implementation include iLQG, AICO, Jacobian pseudo-inverse IK, and a range of sampling based solvers from the [OMPL](http://ompl.kavrakilab.org/) library. The other is the Task Definition which describes the task itself by providing two necessary functions to compute the forward map from Configuration space (say joint angles in IK) to Task space (say end-effector positions in IK). The tasks themselves can describe a complete trajectory. Using the library then involves passing in an initial state and requesting a solution to the problem, which may consist of a single configuration or complete trajectory.

# Prerequisites
* Ubuntu 14.04 (16.04 is periodically tested and compiles, not tested on CI yet)
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/)
* [rosdep](http://wiki.ros.org/rosdep)
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) (```ros-indigo-desktop-full``` is recommended but more minimal versions may be sufficient in conjunction with rosdep)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) > 3.2.7. We recommend system-installing 3.2.10 or newer, for which we provide a [Debian](http://terminator.robots.inf.ed.ac.uk/apt/libeigen3-dev.deb).
* gcc > 4.9 -- the 14.04 system-installed 4.8.4 won't work

# Installation

1. [Create a catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace) or use an existing workspace. [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) is the preferred build system.
1. Clone this repository into the ```src/``` subdirectory of the workspace (any subdirectory below ```src/``` will do): ``git clone git@github.com:ipab-slmc/exotica.git --recursive``.
1. ```cd``` into the the cloned directory.
1. Install dependencies
  1. If running the rosdep for the first time start by running:
     ```sudo rosdep init```
  1. ```rosdep update ; rosdep check --from-paths ./ -s -i ```
1. Compile the code ```catkin build -s```.
1. Source the config file (ideally inside ```~/.bashrc```): ```source path_to_workspace/devel/setup.bash```. You may have to source the config file from your installspace if your workspace is configured for installation.

# Demos
Have a look at ```examples/exotica_examples```.
If you have sourced the workspace correctly you should be able to run any of the demos:

```
roslaunch exotica_examples Core.launch
roslaunch exotica_examples AICOplanner.launch
roslaunch exotica_examples GenericInitialization.launch
roslaunch exotica_examples ManualInitialization.launch
roslaunch exotica_examples OMPLplanner.launch
roslaunch exotica_examples XMLInitialization.launch
```

     
