******************
Quickstart: Python
******************

Quick IK trajectories without ROS
=================================

For this quick tutorial we'll be using the `example\_ik\_noros.py <https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/scripts/example_ik_noros.py>`_ script to generate some simple trajectories.

.. rubric:: CODE

.. code-block:: python

	#!/usr/bin/env python
	print('Start')
	import pyexotica as exo
	from numpy import array
	from numpy import matrix
	import math
	from time import sleep

	def figureEight(t):
		return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

	(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_solver_demo.xml')
	problem = exo.Setup.createProblem(prob)
	solver = exo.Setup.createSolver(sol)
	solver.specifyProblem(problem)

	dt=1.0/20.0
	t=0.0
	q=array([0.0]*7)
	print('Publishing IK')
	timer = exo.Timer()
	while True:
		timer.reset()
		problem.setGoal('Position',figureEight(t))
		problem.startState = q
		q = solver.solve()[0]
		print('Solution found in '+str(timer.getDuration())+'s '+str(q))
		sleep(dt)
	t=t+dt

.. rubric:: CODE EXPLAINED

Setting Start Position
======================

Here, `q` is the starting configuration of the robot which is passed to the solver. Here we set all joints to an initial 0 position:

.. code-block:: python

	q=array([0.0]*7)

To alter the start position, specify a 1*n vector of initial joint angles, where n is the number of DOF. `q` will remain the vector of joint angles whose values will be replaced with the solution when we call the solve function: 

.. code-block:: python

	q = solver.solve()[0]


Setting the Goal 
================

The goal is set using the problem's setGoal function:

.. code-block:: python

	problem.setGoal('Position',figureEight(t))


Two arguments need to be passed into the function: the name of the task map, which here, is `'Position'` (we'll look at where we get this from later) and an array containing the Cartesian goal coordinates (3 Position, 3 Orientation).

In the example, the goal is set to follow the shape of a figure of 8, defined earlier in the script. This works by calling solve on a loop. To set a fixed goal, we might see the following:

.. code-block:: python

	goal = array([0.5,0.2,0.3,0.1,0.8,0.5])
	problem.setGoal('Position',goal)

Where goal values are set to arbitrary coordinates.

The task map named `Position` is set in the XML setup file for this example. We look more at task maps in the task maps tutorial.

Expected Output
===============

When we run the script using `python example_ik_noros.py`, we see the following result in the terminal:

    [ INFO] [1505929027.003072480]: Loading robot model 'lwr'...
    
    Publishing IK
    
    Solution found in 0.000280003s [ -6.18621023e-15  -9.09070542e-02  -8.66069432e-15   9.26337047e-02 -1.44344905e-14  -1.00000000e-01   0.00000000e+00]
    
    Solution found in 0.000178323s [-0.01107742 -0.07860809  0.0059596   0.1926337   0.00494248 -0.07840889 0.]
    ...

This shows us the robot model we are using and then displays the joint angles for each of the 7 joints on our example robot after the time it took to solve the problem.


Problem and Solution Setup 
==========================

To prepare EXOTica for solving motion plans, we must first specify what problem we want to solve (e.g. end pose problem, optimisation problem) and which solver we will use to solve it (e.g. end pose problems be solved by the IKSolver, optimisation problems can be solved by the AICOSolver). The basics will be explained below:

.. code-block:: python

	(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_solver_demo.xml')
	problem = exo.Setup.createProblem(prob)
	solver = exo.Setup.createSolver(sol)
	solver.specifyProblem(problem)


First, we load the XML file

.. code-block:: python

	(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_solver_demo.xml')


which contains a description of the robot, the problem and solver we are using as well as any task maps. The `exo.Initializers.loadXMLFull` command returns the details of the problem and solver, which then need to be sent to EXOTica:

.. code-block:: python

	problem = exo.Setup.createProblem(prob)
	solver = exo.Setup.createSolver(sol)

These form the basis of the way we set the goal and solve the problem later on in the script (e.g. `problem.setGoal('Position',figureEight(t))`).

The last step in this setup is to send the problem to the solver: 

.. code-block:: python

	solver.specifyProblem(problem)

This sends the robot information, task maps and all other problem information to the solver to be used in computing the motion plan.

With this information you are now able to experiment with the example code to familiarise yourself with how these functions effect the action of EXOTIca.

Alternative Python Initialization
=================================

Instead of using the ``loadXMLFull`` method for initialization, we can use the ``loadSolver`` method. For example:

.. code-block:: python

	solver=exo.Setup.loadSolver('{exotica_examples}/resources/configs/aico_solver_demo_eight.xml')
	problem = solver.getProblem()

which loads both the solver and problem from the XML in one method. We then extract the problem 
from the solver. 

Quick IK trajectories with ROS
==============================

The ROS demo script works in exactly the same way as the non-ROS script shown above, but with the addition of the motion plan being published to a ROS topic for visualisation in RVIZ.

For this part of the tutorial, we'll be looking at the 'example_ik.py<https://github.com/ipab-slmc/exotica/blob/master/examples/exotica_examples/scripts/example_ik.py>'_ script.

For details on setting initial joint angles and goal states - see the section above. This section will focus on the additional functionality which allows visualisation in RVIZ.

.. rubric:: CODE

.. code-block:: python

	#!/usr/bin/env python

	import pyexotica as exo
	from numpy import array
	from numpy import matrix
	import math
	from pyexotica.publish_trajectory import *
	from time import sleep

	def figureEight(t):
		return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

	exo.Setup.initRos()
	(sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_solver_demo.xml')
	problem = exo.Setup.createProblem(prob)
	solver = exo.Setup.createSolver(sol)
	solver.specifyProblem(problem)

	dt=0.002
	t=0.0
	q=array([0.0]*7)
	print('Publishing IK')
	while not is_shutdown():
		problem.setGoal('Position',figureEight(t))
		problem.startState = q
		q = solver.solve()[0]
		publishPose(q, problem)    
		sleep(dt)
	t=t+dt

Visualization
=============

In the code we see the function `put code here`. This is a native function in EXOTica which publishes 'ROS TF<http://docs.ros.org/api/geometry_msgs/html/msg/Transform.html>'_  messages to RViz. 

By opening RVIZ and subscribing to the appropriate topic, we will be able to visualise the example arm moving through its motion plan as represented by the TF frames. 

*NOTE: Remember to run roscore before running the script*
