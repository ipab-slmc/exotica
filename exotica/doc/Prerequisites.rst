************
Requirements 
************

URDF File
=========

EXOTica extracts the dimensions and details of the robot from the robot's 
URDF configuration file. To start generating motion plans for your own robot, 
a URDF file is requried.

The URDF file for the`LWR\_simplified <https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/resources/robots/lwr_simplified.urdf>`__
kuka arm included in the EXOTica examples file will be used throughout these tutorials.

SRDF File
=========

In addition to the URDF file, an SRDF file is also required. This
contains semantic information about the robot extracted from the URDF.
EXOTica extracts joint dimensions, limits, DH parameters and self
collision matrices from the SRDF file. Follow the `MoveIt! setup
assistant <http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html>`__ tutorial to generate an SRDF file. The SRDF file for the
`LWR\_simplified <https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/resources/robots/lwr_simplified.srdf>`__
kuka arm included in the source files will be used throughout these
tutorials.
 
We will look at how to direct EXOTica to the URDF and SRDF files in the initialisation section.

CMakeLists.txt & package.xml
============================

Add the following lines to the CMakeLists.txt file of any package that uses EXOTica. 
In the ``find_package(catkin REQUIRED COMPONENTS)`` section, 
add 

.. code:: xml

    exotica
    task_map

Also, add the following to ``package.xml``:

.. code:: xml

    <depend>task_map</depend>