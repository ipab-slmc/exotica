******************
Quickstart: Python
******************

Quick IK trajectories without ROS
=================================

For this quick tutorial we'll be using the `example\_ik\_noros.py <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/scripts/example_ik_noros>`_ script to generate some simple trajectories.

.. rubric:: CODE

.. code-block:: python

    #!/usr/bin/env python
    from __future__ import print_function
    import signal
    import pyexotica as exo
    from pyexotica.publish_trajectory import sig_int_handler
    from numpy import array
    import math
    from time import sleep

    def figure_eight(t):
        return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])


    solver = exo.Setup.load_solver('{exotica_examples}/resources/configs/example_ik.xml')
    problem = solver.get_problem()

    dt = 1.0/20.0
    t = 0.0
    q = array([0.0]*7)
    print('Publishing IK')
    timer = exo.Timer()
    signal.signal(signal.SIGINT, sig_int_handler)
    while True:
        try:
            timer.reset()
            problem.set_goal('Position', figure_eight(t))
            problem.start_state = q
            q = solver.solve()[0]
            print('Solution found in ' + str (timer.get_duration()) + 's ' + str(q))
            sleep(dt)
            t = t + dt
        except KeyboardInterrupt:
    break

.. rubric:: CODE EXPLAINED

Problem and Solution Setup 
--------------------------

To prepare EXOTica for solving motion plans, we must first specify what problem we want to solve and which solver we will use to solve it. First, we load the solver and the problem from an XML file:

.. code-block:: python

    solver = exo.Setup.load_solver('{exotica_examples}/resources/configs/example_ik.xml')

The solver is ready at this point.
However, for convenince we extract the problem from the solver as well:

.. code-block:: python

    problem = solver.get_problem()


We can modify the problem, e.g., by setting the state configuration and a new goal.

Setting Start Position
----------------------

Here, `q` is the starting configuration of the robot which is passed to the solver. Here we set all joints to an initial 0 position:

.. code-block:: python

    q=array([0.0]*7)

To alter the start position, specify a 1*n vector of initial joint angles, where n is the number of DOF. `q` will remain the vector of joint angles whose values will be replaced with the solution when we call the solve function: 

.. code-block:: python

    q = solver.solve()[0]


Setting the Goal 
----------------

The goal is set using the problem's set_goal function:

.. code-block:: python

    problem.set_goal('Position', figure_eight(t))


Two arguments need to be passed into the function: the name of the task map, which here, is `'Position'` (we'll look at where we get this from later) and an array containing the Cartesian goal coordinates (3 Position, 3 Orientation).

In the example, the goal is set to follow the shape of a figure of 8, defined earlier in the script. This works by calling solve on a loop. To set a fixed goal, we might see the following:

.. code-block:: python

    goal = array([0.5,0.2,0.3,0.1,0.8,0.5])
    problem.set_goal('Position', goal)

Where goal values are set to arbitrary coordinates (`[z, y, z, 0, 0, 0]`).

The task map named `Position` is set in the XML setup file for this example. We look more at task maps in the task maps tutorial.

.._expected-output-python:
Expected Output
---------------

When we run the script using ``python example_ik_noros.py``, we see the following result in the terminal:

::

    Publishing IK
    
    Solution found in 0.000280003s [ -6.18621023e-15  -9.09070542e-02  -8.66069432e-15   9.26337047e-02 -1.44344905e-14  -1.00000000e-01   0.00000000e+00]
    
    Solution found in 0.000178323s [-0.01107742 -0.07860809  0.0059596   0.1926337   0.00494248 -0.07840889 0.]
    ...

This shows us the robot model we are using and then displays the joint angles for each of the 7 joints on our example robot after the time it took to solve the problem.

Quick IK trajectories with ROS
------------------------------

The ROS demo script works in exactly the same way as the non-ROS script shown above, but with the addition of the motion plan being published to a ROS topic for visualisation in RVIZ.

For this part of the tutorial, we'll be looking at the `example_ik.py <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/scripts/example_ik>`_ script.

.. rubric:: CODE

.. code-block:: python

    #!/usr/bin/env python

    import pyexotica as exo
    from numpy import array
    from numpy import matrix
    import math
    from pyexotica.publish_trajectory import *
    from time import sleep
    import signal


    def figure_eight(t):
        return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])


    exo.Setup.init_ros()
    solver = exo.Setup.load_solver(
        '{exotica_examples}/resources/configs/example_ik.xml')
    problem = solver.get_problem()

    dt = 0.002
    t = 0.0
    q = array([0.0] * 7)
    print('Publishing IK')
    signal.signal(signal.SIGINT, sig_int_handler)
    while True:
        try:
            problem.set_goal('Position', figure_eight(t))
            problem.start_state = q
            q = solver.solve()[0]
            publish_pose(q, problem)
            sleep(dt)
            t = t + dt
        except KeyboardInterrupt:
    break

Visualization
-------------

In the code we see the function ``problem.get_scene().get_kinematic_tree().publish_frames()``. This is a native function in EXOTica which publishes `ROS tf <http://docs.ros.org/api/geometry_msgs/html/msg/Transform.html>`_  messages to RViz.

By opening RVIZ and subscribing to the appropriate topic, we will be able to visualise the example arm moving through its motion plan as represented by the TF frames. The provided launch file will do this:

::

    roslaunch exotica_examples python_ik.launch
