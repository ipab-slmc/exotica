************
Installation
************

Prerequisites
=============

* Ubuntu 14.04 with ROS Indigo or 16.04 with ROS Kinetic 
* `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/>`_ (``catkin_make`` is no longer supported)
* `rosdep <http://wiki.ros.org/rosdep>`_ to fetch dependencies 
* `Eigen <http://eigen.tuxfamily.org/index.php?title=Main_Page>`_ >3.2.7. We recommend system-installing 3.2.10 or newer, for which we provide a `Debian <http://terminator.robots.inf.ed.ac.uk/apt/libeigen3-dev.deb>`_.
* `gcc 4.9 or newer <https://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu>`_ -- the 14.04 system-installed 4.8.4 won't work

Installation
============

1. `Create a catkin workspace <https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace>`_ or use an existing workspace. `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/>`_ is the only supported build system.
2. Clone this repository into the ``src/`` subdirectory of the workspace (any subdirectory below ``src/`` will do): ``git clone git@github.com:ipab-slmc/exotica.git --recursive``.
3. ``cd`` into the the cloned directory.
4. Install dependencies
	1. If running the rosdep for the first time start by running: ``sudo rosdep init``
	2. Update the database and install the dependencies: ``rosdep update ; rosdep install --from-paths ./ -i``
5. Compile the code ``catkin build -s``.
6. Source the config file (ideally inside ``~/.bashrc``): ``source path_to_workspace/devel/setup.bash``. You may have to source the config file from your install-space if your workspace is configured for installation.

Demos
=====

Have a look at ``examples/exotica_examples``.
If you have sourced the workspace correctly you should be able to run any of the demos:

.. code-block:: bash

	roslaunch exotica_examples Core.launch
	roslaunch exotica_examples AICOplanner.launch
	roslaunch exotica_examples GenericInitialization.launch
	roslaunch exotica_examples ManualInitialization.launch
	roslaunch exotica_examples OMPLplanner.launch
	roslaunch exotica_examples XMLInitialization.launch
