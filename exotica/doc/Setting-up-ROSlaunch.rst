********************
Setting up ROSlaunch
********************

Generic ROSlaunch Structure
===========================

Using ROSlaunch to launch your EXOTica script makes directing locating
your URDF and SRDF scripts among other things a lot easier.

As we can see in the roslaunch file for the manually initialised example
`here <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/launch/ManualInitialization.launch>`__
and seen below, there are a few essential elements to constructing a
ROSlaunch file.

.. code:: xml

    <launch>

      <arg name="debug" default="false" />
      <arg unless="$(arg debug)" name="launch_prefix" value="" />
      <arg     if="$(arg debug)" name="launch_prefix" value="xterm -e gdb --args" />

      <param name="robot_description" textfile="$(find exotica_examples)/resources/lwr_simplified.urdf" />
      <param name="robot_description_semantic" textfile="$(find exotica_examples)/resources/lwr_simplified.srdf" />

      <node launch-prefix="$(arg launch_prefix)" pkg="exotica_examples" type="ManualInitialization" name="ExoticaManualInitializationExampleNode" output="screen" />

      <node name="rviz" pkg="rviz" type="rviz" respawn="false"  args="-d $(find exotica_examples)/resources/rviz.rviz" />
    </launch>

Now we will go through the necessary components of this file.

Debug Param
~~~~~~~~~~~

EXOTica looks for a ROSParam named ``"debug"``. Here we set the default
value to false. This can be changed according to your needs.

.. code:: xml

      <arg name="debug" default="false" />
      <arg unless="$(arg debug)" name="launch_prefix" value="" />
      <arg if="$(arg debug)" name="launch_prefix" value="xterm -e gdb --args" />

The line after instructs ROSlaunch to change the value of debug if we
specify the arg name ``debug`` when launching the file in the terminal.
This will look like this:

.. code:: shell

    roslaunch exotica_examples ManualInitialization.launch debug:=true

Finally, we specify the debug terminal to be launched. Here we use
xterm, but can be switched accordingly.

URDF and SRDF specification
~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the example launch file, we see two parameters specified:
``robot_description`` and ``robot_description_semantic``. These are the
locations from which EXOTica reads the file names for the URDF and SRDF
files respectively. These parameter names are fixed and the file names
should be attached to these exact names.

.. code:: xml

      <param name="robot_description" textfile="$(find exotica_examples)/resources/lwr_simplified.urdf" />
      <param name="robot_description_semantic" textfile="$(find exotica_examples)/resources/lwr_simplified.srdf" />

Here the file names are specified through the ROS packages in which they
are found (``$(find exotica_examples)``) and then directed to the
specific files.

If your robot description files are not stored in ROS package, the
absolute file name can also be used, though this is not recommended.
This is done using:
``"file:///home/username/path/path/robot_name.srdf"``

Specify EXOTica code location
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We must then direct the ROSlaunch file to EXOTica code that we made
earlier in the tutorial.

.. code:: xml

    <node launch-prefix="$(arg launch_prefix)" pkg="exotica_examples" type="ManualInitialization" name="ExoticaManualInitializationExampleNode" output="screen" />

In order to do this, we specify the package in which you put your
EXOTica code. Here we stored it in the ``exotica_examples`` package. In
the ``type`` argument, we specify the name of the EXOTica code that we
assigned to it in the ``CMakeLists.txt`` file. Here, we saved it under
``ManualInitialization``, so that's what we specify here.

Finally, we specify the name for the ROS node and the output for the
ROSlaunch file.

Starting RVIZ
~~~~~~~~~~~~~

We can also start RVIZ from here and direct it to a specific .rviz save
file:

.. code:: xml

    <node name="rviz" pkg="rviz" type="rviz" respawn="false"    args="-d $(find exotica_examples)/resources/rviz.rviz" />

XML Parameters
==============

When initialising using XML, we also need to use the ROSlaunch file to
direct EXOTica to the XML file. We can do this by altering the
``<node>`` section to add the params:
``ConfigurationFile``,\ ``Solver``,\ ``Problem`` as such:

.. code:: xml


      <node launch-prefix="$(arg launch_prefix)" pkg="exotica_examples" type="XMLInitialization" name="ExoticaXMLInitializationExampleNode" output="screen">
        <param name="ConfigurationFile" type="string" value="$(find exotica_examples)/resources/ik_solver_demo.xml" />
        <param name="Solver" type="string" value="MySolver" />
        <param name="Problem" type="string" value="MyProblem" />
    </node>

Which will then be read by the XML initialiser in your script. Simply
change the value of the ``ConfigurationFile`` parameter to set this
parameter.
