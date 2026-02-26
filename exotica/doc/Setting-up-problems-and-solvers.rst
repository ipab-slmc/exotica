*******************************
Setting up Problems and Solvers
*******************************

To solve motion plans with EXOTica, we create a problem, 
send it to the solver and then solve said problem. We will look now in more 
detail about setting up problems, solvers.

Here we will run through the initialization of a problem, setting up problems, 
solvers and solving for motion plans. If you prefer to initialize using XML, 
skip to the problem setup part of the tutorial. 

For this tutorial, we will use the `generic.cpp <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/src/generic.cpp>`__ 
example file as a guide. The code is displayed below. 


.. code-block:: cpp

    #include <exotica_core/exotica_core.h>

    // Manual initialization requires dependency on specific solvers and task maps:
    #include <exotica_ik_solver/IKSolver_initializer.h>
    #include <task_map/EffFrame_initializer.h>

    using namespace exotica;

    void run()
    {
        Server::InitRos(std::sharedPtr<ros::NodeHandle>(new ros::NodeHandle("~")));

        // Scene using joint group 'arm'
        SceneInitializer scene("MyScene", "arm", false, "", "{exotica_examples}/resources/robots/lwr_simplified.urdf", "{exotica_examples}/resources/robots/lwr_simplified.srdf");
        // End-effector task map with two position frames
        EffFrameInitializer map("Position", false,
                                {FrameInitializer("lwr_arm_6_link", Eigen::VectorTransform(0, 0, 0, 0.7071067811865476, -4.3297802811774664e-17, 0.7071067811865475, 4.3297802811774664e-17))});
        // Create a task using the map above (goal will be specified later)
        Eigen::VectorXd W(7);
        W << 7, 6, 5, 4, 3, 2, 1;

        UnconstrainedEndPoseProblemInitializer problem("MyProblem", scene, false, {map}, W);
        IKSolverInitializer solver("MySolver");
        solver.C = 1e-3;
        solver.MaxIterations = 1;
        solver.MaxStep = 0.1;

        HIGHLIGHT_NAMED("ManualLoader", "Loaded from a hardcoded specialized initializer.");

        // Initialize

        PlanningProblemPtr any_problem = Setup::CreateProblem(problem);
        MotionSolverPtr any_solver = Setup::CreateSolver(solver);

        // Assign the problem to the solver
        any_solver->SpecifyProblem(any_problem);
        UnconstrainedEndPoseProblemPtr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

        // Create the initial configuration
        Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->N);
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
            my_problem->y = {0.6,
                            -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
                            0.5 + sin(t * M_PI * 0.5) * 0.2, 0, 0, 0};

            // Solve the problem using the IK solver
            my_problem->setStartState(q);
            any_solver->Solve(solution);

            double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
            ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving in " << time << "s. Solution [" << solution << "]");
            q = solution.row(solution.rows() - 1);

            my_problem->Update(q);
            my_problem->getScene()->GetKinematicTree().publishFrames();

            ros::spinOnce();
            loop_rate.sleep();
        }

        // All classes will be destroyed at this point.
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "ExoticaManualInitializationExampleNode");
        ROS_INFO_STREAM("Started");

        // Run demo code
        run();

        // Clean up
        // Run this only after all the exoica classes have been disposed of!
        Setup::Destroy();
    }


.. rubric:: CODE EXPLAINED

Including Solvers and Task Maps 
===============================

In this tutorial we will be setting up an ``UnconstrainedEndPoseProblem`` and 
solving it using an IKSolver using the ``EffFrame`` task map. To do this we 
need to include the correct header files:

.. code-block:: cpp

    #include <exotica_core/exotica_core.h>

    // Manual initialization requires dependency on specific solvers and task maps:
    #include <exotica_ik_solver/IKSolver_initializer.h>
    #include <task_map/EffFrame_initializer.h>


Problem definitions are handled in the ``exotica/Exotica.h`` header, so we only need to 
include the ``IKSolverInitializer`` and ``EffFrameInitializer``. 

To use other solvers and task maps just include the appropriate headers in the same 
format. 

ROS Initialization
==================

ROS can either be set up manually or you can use the EXOTica Server. 

To use the EXOTica Server, it needs to be setup using the ``InitRos``:

.. code-block:: cpp

    Server::InitRos(std::sharedPtr<ros::NodeHandle>(new ros::NodeHandle("~")));

Where we provide a name for the ROS node (here we give the name "~")

Scene Setup
===========

To construct a problem, we first need a ``Scene``, a ``map`` and parameters. 

Here we set up the Scene:

.. code-block:: cpp

    // Scene using joint group 'arm'
    SceneInitializer scene("MyScene", "arm", false, "", "{exotica_examples}/resources/robots/lwr_simplified.urdf", "

    
Where we give the SceneInitializer a name ("MyScene"), the name of the joint group ("arm")
which is the same name as the group in the SRDF file. This is followed by the debug argument, 
robot description and the path to the URDF file. 

Map Setup
=========

Next up for the problem setup is the map setup. Here we are interested solving 
an end effector planning problem, so we require an end effector mapping. 

Here we have the option of specifying an interest in the end effector position
using the EffPositionInitializer, the orientation of the end effector using the
EffOrientationInitializer or both, using the EffFrameInitializer. Let's try the 
EffFrame map, where specify a goal for both the position and orientation of the 
end effector

.. code-block:: cpp

    EffFrameInitializer map("Position", false, {FrameInitializer("lwr_arm_6_link", Eigen::VectorTransform(0, 0, 0, 0.7071067811865476, -4.3297802811774664e-17, 0.7071067811865475, 4.3297802811774664e-17))});

Where we pass in:

* Name of the task map ( here we use "Position")
* Debug argument
* FrameInitializer
    - Name of the end effector link (here we use "lwr_arm_6_link" for the lwr_simplified arm)
    - Optional offset from that link 

Problem and Solver Initialization
=================================

Now we have the Scene and Map(s) (multiple maps can be added) initialized, we can set up the 
problem. First we need to create an initializer:

.. code-block:: cpp

 UnconstrainedEndPoseProblemInitializer problem("MyProblem", scene, false, {map}, W);

To do this, we simply call the initializer for the problem we want, giving the initializer a name
(here we use "problem"), then giving the problem a name (here: "MyProblem"). Then pass the scene, 
a debug argument, the map and the parameter (here: "W" as a cost cost weighting for the motion of 
each joint). 

We can then create the initializer for the solver. To do this, we can simply name ("solver") and 
create an initializer, give the solver itself a name ("MySolver") then set the parameters later: 

.. code-block:: cpp

    IKSolverInitializer solver("MySolver");
    solver.C = 1e-3;
    solver.MaxIterations = 1;
    solver.MaxStep = 0.1;

or parameters can be set in arguments to the initializer. See 
`initialization files <https://github.com/ipab-slmc/exotica/tree/master/exotica_core/init>`_ for
details of each solver's options. 

The next step is to send the problem and solver to the Planning and Motion Solver pointer
containers. Here use the name of the initializer, not the names of the problems and solvers.

.. code-block:: cpp

    PlanningProblemPtr any_problem = Setup::CreateProblem(problem);
    MotionSolverPtr any_solver = Setup::CreateSolver(solver);

Sending Problem to Solvers
==========================

We now have our problem set up, containing all the information about the robot, task etc. 
and we have a solver setup, waiting to solve some motion plans, but they don't know about
each other. Let's now send the problem to the solver:

.. code-block:: cpp

    any_solver->SpecifyProblem(any_problem);
    UnconstrainedEndPoseProblemPtr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

When sending the problem to the solver, we use the pointers we created in the last step, named: "any_problem" and 
"any_solver": 

.. code-block:: cpp

    any_solver->SpecifyProblem(any_problem);

Finally, we pop the problem back into a specific problem pointer to be used later:

.. code-block:: cpp

    UnconstrainedEndPoseProblemPtr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

This procedure applies to all problems and solvers, but the parameters for each will vary. 
Please refer to the `initialization files <https://github.com/ipab-slmc/exotica/tree/master/exotica_core/init>`_
for setup details for each. 

Also, multiple problems can be initialized and sent to solvers in a single script, they just need unique names
to do so. 

And that's the problem set up. We can now start to use EXOTica to solve motion plans, which we will look
at in the next tutorial. 