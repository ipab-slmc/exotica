..  _installation:

******************
Installing EXOTica
******************

Prerequisites
=============

For all installations:
 * Ubuntu 16.04 with ROS Kinetic or Ubuntu 18.04 with ROS Melodic. Ubuntu 14.04 with ROS Indigo may continue to work but is not officially supported or covered by continuous integration any longer.
 * `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/>`_ (``catkin_make`` is no longer supported)
 * `rosdep <http://wiki.ros.org/rosdep>`_
 * `ROS <http://wiki.ros.org/Installation>`_ (``ros-[release]-desktop`` is recommended but more minimal versions work in conjunction with rosdep)

ROS Indigo (Ubuntu 14.04) -- retired since September 2018:
 * For Eigen, we recommend system-installing 3.2.10 or newer, for which we provide a `Debian <http://terminator.robots.inf.ed.ac.uk/apt/libeigen3-dev.deb>`_.
 * `gcc >4.9 <https://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu>`_ -- the 14.04 system-installed 4.8.4 won't work. On Ubuntu 16.04 and newer, you can use the system-provided gcc/g++.
 * For compiling ``fcl_catkin``, you need to add a PPA for ``libccd-dev``.

Installation from binaries
==========================

We regularly release EXOTica for ROS Kinetic and ROS Melodic. You can install EXOTica from binary via

::

	sudo apt install ros-[distro]-exotica ros-[distro]-exotica-examples

Installation from source
========================

Clean installation:
 1. `Create a catkin workspace <https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace>`_ or use an existing workspace. `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/>`_ is the preferred build system.
 2. Clone this repository into the ``src/`` subdirectory of the workspace (any subdirectory below ``src/`` will do): ``git clone git@github.com:ipab-slmc/exotica.git``.
 3. ``cd`` into the the cloned directory.
 4. Install dependencies

  1. If running rosdep for the first time start by running:
     ``sudo rosdep init``
  2. ``rosdep update ; rosdep install --from-paths ./ -iy``

 5. Compile the code ``catkin build``.
 6. Source the config file (ideally inside ``~/.bashrc``): ``source path_to_workspace/devel/setup.bash``. You may have to source the config file from your installspace if your workspace is configured for installation.

Demos
=====

Have a look at the ``exotica_examples`` package.
If you have sourced the workspace correctly you should be able to run any of the demos, e.g.:

::

	roslaunch exotica_examples cpp_ik_minimal.launch
	roslaunch exotica_examples cpp_core.launch
	roslaunch exotica_examples cpp_aico.launch
	roslaunch exotica_examples python_ompl.launch
	roslaunch exotica_examples python_attach.launch
	roslaunch exotica_examples python_collision_distance.launch
	roslaunch exotica_examples python_sphere_collision.launch
