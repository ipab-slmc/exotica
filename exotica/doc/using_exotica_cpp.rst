********************************
Solving planning problems in C++
********************************

This part of the tutorial assumes that you have completed the previous 
step. We will work off the same code and get on with solving some motion 
plans.

We will continue to use the exotica example found in
``manual.cpp`` as a reference, starting after we sent the problem to 
the solver:

.. code-block:: c++

    ...

    // Continued from initialization
    // Create the initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->GetScene()->GetNumControlledJoints());
    Eigen::MatrixXd solution;

    ROS_INFO_STREAM("Calling solve() in an infinite loop");

    double t = 0.0;
    ros::Rate loop_rate(500.0);
    ros::WallTime init_time = ros::WallTime::now();

    while (ros::ok())
    {
        ros::WallTime start_time = ros::WallTime::now();

        // Update the goal if necessary
        // e.g. figure eight
        t = ros::Duration((ros::WallTime::now() - init_time).toSec()).toSec();
        my_problem->y << 0.6,
                        -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
                            0.5 + sin(t * M_PI * 0.5) * 0.2;

        // Solve the problem using the IK solver
        any_solver->Solve(solution);

        double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
        ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving in "<<time<<"s. Solution ["<<solution<<"]");
        q = solution.row(solution.rows() - 1);

        my_problem->Update(q);
        my_problem->GetScene()->GetKinematicTree().PublishFrames();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // All classes will be destroyed at this point.
    ...

.. rubric:: CODE EXPLAINED

.._setting-start-position-cpp:
Setting Start Position
======================

Firstly, we must set up a vector of initial joint variables to be fed
into the solver. Here we initialize the joints to zero angles:

.. code-block:: c++

        Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->getScene()->getNumControlledJoints());

Using the robot description to determine the size
of the vector (``any_problem->getScene()->getNumControlledJoints()``). This vector
can naturally be changed to whatever joint configuration is required for
your motion planning purposes. for example, after instantiating ``q``:

.. code-block:: c++

    q << joint_start[0],joint_start[1],joint_start[2],joint_start[3],joint_start[4],joint_start[5],joint_start[6];

and so on. This init joint configuration vector will be sent to the
solver later.

Solution Container
==================

When we do call the solver, we will get the motion plan back as an
n\*DOF matrix, where n is the number of steps along the trajectory. A
dynamic matrix container will need to be created to hold this.
This is created in the next line:

.. code-block:: c++

    Eigen::MatrixXd solution;

Now we have a starting point and something to hold the trajectory, we
need somewhere to go. It's time to set the goal.

Goal Setting
============

Methods of goal setting vary according to the problem and your
requirements. Two methods exist for the IK_solver. The method used in
the tutorial specifies the goal for all IK_solver task maps by setting
the ``y`` value to the desired goal:

.. code-block:: cpp

            my_problem->y << 0.6,  // X Position
                            -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1, // Y Position
                             0.5 + sin(t * M_PI * 0.5) * 0.2; // Z Position

NOTE: To set the goal for an individual map, use the ``SetGoal()``
function. This requires the name of the task map and a Eigen vector
containing the Cartesian coordinates of the goal:

.. code-block:: cpp

    Eigen::VectorXd goal(3);

    goal << 0.6,0.8,0.5;

    my_problem->SetGoal("Position",goal);

Now the initial joint positions have been set, we have the solution
container and have set the goal, we are ready to solve the problem using
the ``any_solver`` container in which we stored the generic solver in
earlier, passing the initial joint states ``q`` and the trajectory
holder:

Solving Problems
================

.. code-block:: c++

        any_solver->Solve(solution);

Solution Format
===============

Now we have a solution to our problem. But what does it look like?

.. code-block:: shell

    [ INFO] [1501240815.111167097]: Finished solving in 3.085e-05s. Solution [  -0.109557   -0.653855  -0.0687444     1.28515 1.06079e-17           0           0]

When using the IK_solver as in this tutorial and we set the ``MaxIterations`` to a
low number, we get single step solution to the IK problem, as shown above -
this is what you would expect to see if you run this code;
it shows a vector of angles, one column  for each joint in our
robot. Each entry a joint configuration in radians, which will result in
the end effector reaching the desired target. The rows of the output
represent the positional steps each joint must pass through to reach 
the end effector goal. When using a higher ``MaxIterations`` setting, the number 
of rows in your motion plan would likely increase. 

When using other problems or a different configuration of the
``UnconstrainedEndPoseProblem``, trajectories will start to look a
little more substantial. The output below shows the format of a solution
after being solved by the ``OMPLSolver``. Note that this solution was
computed for a 6DOF robot and thus contains 6 columns. The first row
represents the initial joint configuration, which here we set to zeros.
The final row shows the configuration of the robot which allows the
end-effector to reach the goal. The intermediate rows are the positional
configurations that transfer the arm from start to end.

::

    Solution:
       [       0            0            0            0              0     1.63042e-322
        0.134729   0.00623148  -0.00439002     -0.0770144   -6.41669e-18   1.63042e-322
        0.269458     0.012463  -0.00878005     -0.154029    -1.28334e-17   1.63042e-322
        0.404187    0.0186944   -0.0131701     -0.231043    -1.92501e-17   1.63042e-322
        0.538915    0.0249259   -0.0175601     -0.308058    -2.56668e-17   1.63042e-322
        0.673644    0.0311574   -0.0219501     -0.385072    -3.20835e-17   1.63042e-322
        0.808373    0.0373889   -0.0263401     -0.462086    -3.85001e-17   1.63042e-322
        0.943102    0.0436204   -0.0307302     -0.539101    -4.49168e-17   1.63042e-322
         1.07783    0.0498519   -0.0351202     -0.616115    -5.13335e-17   1.63042e-322
         1.21256    0.0560833   -0.0395102     -0.69313     -5.77502e-17   1.63042e-322
         1.34729    0.0623148   -0.0439002     -0.770144    -6.41669e-17   1.63042e-322
         1.48202    0.0685463   -0.0482903     -0.847158    -7.05836e-17   1.63042e-322
         1.61675    0.0747778   -0.0526803     -0.924173    -7.70003e-17   1.63042e-322
         1.75148    0.0810093   -0.0570703     -1.00119     -8.3417e-17    1.63042e-322
          1.8862    0.0872407   -0.0614603     -1.0782      -8.98337e-17   1.63042e-322
         2.02093    0.0934722   -0.0658504     -1.15522     -9.62504e-17   1.63042e-322
         2.15566    0.0997037   -0.0702404     -1.23223     -1.02667e-16   1.63042e-322
         2.29039     0.105935   -0.0746304     -1.30924     -1.09084e-16   1.63042e-322
         2.42512     0.112167   -0.0790204     -1.38626     -1.155e-16     1.63042e-322
         2.55985     0.118398   -0.0834105     -1.46327     -1.21917e-16   1.63042e-322
         2.69458      0.12463   -0.0878005     -1.54029     -1.28334e-16   1.63042e-322 ]

Publishing to RVIZ
==================

We've set up or problem, solver and the rest and got our motion plan.
EXOTica has the functionality to visualize this in RVIZ, so you can see
your plan in action. The parts of the example code we are yet to mention
deals with this and we'll go through it now.

As we cycle through our motion plan, we can update the joint states:

.. code-block:: c++

            q = solution.row(solution.rows() - 1);

and we send them to the problem:

.. code-block:: c++

            my_problem->Update(q);

Now we can publish those frames to the /joint\_states topic to be read
by RVIZ:

.. code-block:: c++

            my_problem->GetScene()->GetKinematicTree().PublishFrames();

RVIZ can either be set-up manually or via a
`roslaunch <Setting-up-ROSlaunch.html>`__
file.
