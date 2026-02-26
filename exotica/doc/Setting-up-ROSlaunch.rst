********************
Setting up ROSlaunch
********************

Generic ROSlaunch Structure
===========================

Using ROSlaunch to launch your EXOTica script makes locating
your URDF and SRDF files and keeping everything together much easier.

As we can see in the roslaunch file for the manually initialized example
`here <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/launch/cpp_init_generic.launch>`__
and seen below, there are a few essential elements to constructing an EXOTica
ROSlaunch file.

.. code-block:: xml

    <launch>

        <arg name="debug" default="false" />
        <arg unless="$(arg debug)" name="launch_prefix" value="" />
        <arg     if="$(arg debug)" name="launch_prefix" value="xterm -e gdb --args" />

        <param name="robot_description" textfile="$(find exotica_examples)/resources/robots/lwr_simplified.urdf" />
        <param name="robot_description_semantic" textfile="$(find exotica_examples)/resources/robots/lwr_simplified.srdf" />

        <node launch-prefix="$(arg launch_prefix)" pkg="exotica_examples" type="ManualInitialization" name="ExoticaManualInitializationExampleNode" output="screen" />

        <node name="rviz" pkg="rviz" type="rviz" respawn="false" args="-d $(find exotica_examples)/resources/rviz.rviz" />
    </launch>

.. rubric:: CODE EXPLAINED

Debug Param
===========

EXOTica looks for the "debug" rosparam. Here we set the default
value to false. This can be changed according to your needs.

.. code-block:: xml

      <arg name="debug" default="false" />
      <arg unless="$(arg debug)" name="launch_prefix" value="" />
      <arg if="$(arg debug)" name="launch_prefix" value="xterm -e gdb --args" />

The line after instructs ROSlaunch to change the value of debug if we
specify the arg name ``debug`` when launching the file in the terminal.
This will look like this:

.. code-block:: shell

    roslaunch exotica_examples ManualInitialization.launch debug:=true

Finally, we specify the debug terminal to be launched. Here we use
xterm, but can be changed accordingly.

URDF and SRDF specification
===========================

In the example launch file, we see two parameters specified:
``robot_description`` and ``robot_description_semantic``. These are the
locations from which EXOTica reads the file names for the URDF and SRDF
files respectively. These parameter names are fixed and the file names
should be attached to these exact names.

.. code-block:: xml

      <param name="robot_description" textfile="$(find exotica_examples)/resources/lwr_simplified.urdf" />
      <param name="robot_description_semantic" textfile="$(find exotica_examples)/resources/lwr_simplified.srdf" />

Here the file names are expressed via ROS packages (``$(find exotica_examples)``).
If your robot description files are not stored in ROS package, the
absolute file name can also be used ``"file:///home/username/path/path/robot_name.srdf"``
though this is not recommended 

Specify EXOTica code location
=============================

We must then direct the ROSlaunch file to EXOTica code that we made
earlier in the tutorial.

.. code-block:: xml

    <node launch-prefix="$(arg launch_prefix)" pkg="exotica_examples" type="ManualInitialization" name="ExoticaManualInitializationExampleNode" output="screen" />

To do this, we specify the package in which you put your
EXOTica code. Here we stored it in the ``exotica_examples`` package. In
the ``type`` argument, we specify the name of the EXOTica code that we
assigned to it in the ``CMakeLists.txt`` file. Here, we saved it under
``ManualInitialization``, so that's what we specify here.

Finally, give your code a rosnode name; here we use "ExoticaManualInitializationExampleNode"

Starting RVIZ
=============

We can also start RVIZ from here and direct it to a specific .rviz save
file:

.. code-block:: xml

    <node name="rviz" pkg="rviz" type="rviz" respawn="false"    args="-d $(find exotica_examples)/resources/rviz.rviz" />

XML Parameters
==============

When initializing with XML, ROSlaunch needs to broadcast a ROSparam 
specifying the name of the configuration file (Here we're looking at the `XML launch file <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/launch/cpp_init_generic.launch>`__):

.. code-block:: xml

    <node launch-prefix="$(arg launch_prefix)" pkg="exotica_examples" type="XMLInitialization" name="ExoticaXMLInitializationExampleNode" output="screen">
        <param name="ConfigurationFile" type="string" value="$(find exotica_examples)/resources/configs/ik_solver_demo.xml" />
    </node>

Which will then be read by the XML initializer in your script. Simply
change the value of the ``ConfigurationFile`` parameter to set the
filename.
