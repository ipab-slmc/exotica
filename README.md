# EXOTica 🏝️ [![CI](https://github.com/ipab-slmc/exotica/workflows/CI/badge.svg)](https://github.com/ipab-slmc/exotica/actions?query=workflow%3ACI) [![Analytics](https://ga-beacon.appspot.com/UA-72496975-1/ipab-slmc/exotica/?pixel)](https://github.com/igrigorik/ga-beacon) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/b7d2b85be51645288ff6511f2d3e88a4)](https://app.codacy.com/gh/ipab-slmc/exotica?utm_source=github.com&utm_medium=referral&utm_content=ipab-slmc/exotica&utm_campaign=Badge_Grade_Dashboard)

[Documentation](https://ipab-slmc.github.io/exotica/) - [C++ Doxygen](https://ipab-slmc.github.io/exotica/doxygen_cpp/) - [Python Documentation](https://ipab-slmc.github.io/exotica/Python-API.html) - [Docker](https://github.com/VladimirIvan/exotica-docker-notebook)

The EXOTica library is a general Optimisation Toolset for Robotics platforms, written in C++ with bindings for Python. Its motivation is to provide a more streamlined process for developing algorithms for tasks such as Inverse Kinematics, Trajectory Optimisation, and Optimal Control. Its design advocates:

* **Modularity:** The library is developed in a modular manner making use of C++’s object-oriented features (such as polymorphism). This allows users to define their own components and ’plug them into’ the existing framework. Thus, an engineer does not need to implement a whole system whenever he needs to change a component, but rather can re-implement the specific functionality and as long as he follows certain guidelines, retain the use of the other modules.
* **Extensibility:** The library is also heavily extensible, mainly thanks to the modular design. In addition, the library makes very minimal prior assumptions about the form of the problem so that it can be as general as possible.
* **Integration with ROS:** The library is designed to be fully integrated with ROS allowing to set up, configuration, consuming data from ros topics, and publishing debug display using ROS tools.

The library itself consists of two major specifications, both of which are abstract classes.
The first is the *Motion Solver* which defines the way optimisation should proceed: current implementation include AICO, Jacobian pseudo-inverse IK, and a range of sampling-based solvers from the [OMPL](http://ompl.kavrakilab.org/) library -- in total, more than 60 different motion solvers.
The other is the *Task Definition* which describes the task itself by providing two necessary functions to compute the forward map from Configuration space (say joint angles in IK) to Task space (say end-effector positions in IK). The tasks themselves can describe a complete trajectory. Using the library then involves passing in an initial state and requesting a solution to the problem, which may consist of a single configuration or complete trajectory.
Additionally, users can select different underlying dynamics models by specifying a *DynamicsSolver*. Similarly, different collision checking methods and libraries can be selected using the *CollisionScene* plug-ins.

## Prerequisites
* Ubuntu 16.04 (ROS Kinetic), Ubuntu 18.04 (ROS Melodic), or Ubuntu 20.04 (ROS Noetic).
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) (```catkin_make``` is no longer supported)
* [rosdep](http://wiki.ros.org/rosdep)
* [ROS](http://wiki.ros.org/Installation)

## Installation
### From binary
Exotica is available as binaries for ROS Kinetic and Melodic and can be installed via

```bash
sudo apt install ros-$ROS_DISTRO-exotica
```

### From source
1. [Create a catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace) or use an existing workspace. [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) is the preferred build system.
2. Clone this repository into the ```src/``` subdirectory of the workspace (any subdirectory below ```src/``` will do): ``git clone git@github.com:ipab-slmc/exotica.git``.
3. ```cd``` into the the cloned directory.
4. Install dependencies: ```rosdep update ; rosdep install --from-paths ./ -iry ```
5. Compile the code ```catkin build -s```.
6. Source the config file (ideally inside ```~/.bashrc```): ```source path_to_workspace/devel/setup.bash```. You may have to source the config file from your installspace if your workspace is configured for installation.

## Demos
Have a look at ```exotica_examples```.
If you have sourced the workspace correctly, you should be able to run any of the demos:

```bash
roslaunch exotica_examples cpp_ik_minimal.launch
roslaunch exotica_examples cpp_core.launch
roslaunch exotica_examples cpp_aico.launch
roslaunch exotica_examples python_ompl.launch
roslaunch exotica_examples python_attach.launch
roslaunch exotica_examples python_collision_distance.launch
roslaunch exotica_examples python_sphere_collision.launch
```

## Publications

We have published a [Springer book chapter](https://link.springer.com/chapter/10.1007/978-3-319-91590-6_7) outlining the concept and ideas behind EXOTica and recommend it to new users for getting started in addition to the tutorials and documentation.

> Ivan V., Yang Y., Merkt W., Camilleri M.P., Vijayakumar S. (2019) EXOTica: An Extensible Optimization Toolset for Prototyping and Benchmarking Motion Planning and Control. In: Koubaa A. (eds) Robot Operating System (ROS). Studies in Computational Intelligence, vol 778. Springer, Cham

If you use EXOTica for academic work, please cite the relevant book chapter, a preprint of which is available [here](https://vladimirivan.files.wordpress.com/2018/03/exoticarosbook.pdf):

```bibtex
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
```
