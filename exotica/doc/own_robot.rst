*********************************************
Creating a planner package for your own robot 
*********************************************

To start using EXOTica with a new robot we require the configuration files 
for your robot and some changes to the ROS package that you will be using. 

URDF File
=========

EXOTica extracts the dimensions and details of the robot from the robot's 
URDF configuration file. To start generating motion plans for your own robot, 
a URDF file is required.

The URDF file for the `lwr\_simplified <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/robots/lwr_simplified.urdf>`_
KUKA arm included in the EXOTica examples file will be used throughout these tutorials.

SRDF File
=========

In addition to the URDF file, an SRDF file is also required. This
contains semantic information about the robot extracted from the URDF.
EXOTica extracts joint dimensions, limits, DH parameters and self
collision matrices from the SRDF file. Follow the `MoveIt! setup
assistant <http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html>`__ tutorial to generate an SRDF file. The SRDF file for the
`lwr\_simplified <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/robots/lwr_simplified.srdf>`__
KUKA arm included in the source files will be used throughout these
tutorials.

CMakeLists.txt & package.xml
============================

Add the following lines to the CMakeLists.txt file of any package that uses EXOTica. 
In the ``find_package(catkin REQUIRED COMPONENTS)`` section, 
add 

.. code-block:: cmake

    find_package(catkin REQUIRED COMPONENTS exotica_core)

Also, add the following to ``package.xml`` for a c++ package:

.. code-block:: xml

    <depend>exotica_core</depend>

or the following for a python package:

.. code-block:: xml

    <depend>exotica_python</depend>

