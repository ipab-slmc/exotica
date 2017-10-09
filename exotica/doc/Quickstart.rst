**********
Quickstart
**********

Quick IK trajectories without ROS
=================================

For this quick tutorial we'll be using the
`example\_ik\_noros.py <https://github.com/ipab-slmc/exotica/blob/master/exotica_python/scripts/example_ik_noros.py>`__
script to generate some simple trajectories.

Code
~~~~

.. code:: python

    #!/usr/bin/env python
    print('Start')
    import pyexotica as exo
    from numpy import array
    from numpy import matrix
    import math
    from time import sleep

    def figureEight(t):
        return array([0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0, 0, 0])

    (sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica')+'/resources/configs/ik_solver_demo.xml')
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

Code explained
~~~~~~~~~~~~~~

Goal Setting
^^^^^^^^^^^^

To set the goal, we need to look at the
``problem.setGoal('Position',figureEight(t))`` line in our code. This
requires 2 arguments: the name of the task map and a numpy array for the
goal containing 3 pose and 3 orientation arguments.

When running the script using ``python example_ik_noros.py``, we see the
following result in the terminal:

::

    [ INFO] [1505929027.003072480]: Loading robot model 'lwr'...
    Publishing IK
    Solution found in 0.000280003s [ -6.18621023e-15  -9.09070542e-02  -8.66069432e-15   9.26337047e-02 -1.44344905e-14  -1.00000000e-01   0.00000000e+00]
    Solution found in 0.000178323s [-0.01107742 -0.07860809  0.0059596   0.1926337   0.00494248 -0.07840889 0.]
    ...

This shows us the robot model we are using and then displays the joint
angles for each of the 7 joints on our example robot after the time it
took to solve the problem. # Quick IK trajectories with ROS ##
Visualising
