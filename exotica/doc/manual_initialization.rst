*********************
Manual Initialization
*********************

Manual initialization encapsulates all initialisation and handling of
EXOTica within C++ or Python code, with no external XML files. This could
be preferred if your project has been finalized or if you prefer to have all your EXOTica code in one place.

An `example of manual initialization <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/scripts/example_ik_manual_initialization>`__  in Python can be found in the ``scripts`` directory of the ``exotica_examples`` package. In this tutorial, we will use the `example of manual initialization <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/src/generic.cpp>`__ for the UnconstrainedEndPoseProblem written in C++, found in the ``src`` directory of the ``exotica_examples`` package:

.. code-block:: c++

    #include <exotica_core/exotica_core.h>

    // Manual initialization requires dependency on specific solvers and task maps:
    #include <exotica_ik_solver/ik_solver_initializer.h>
    #include <task_map/eff_frame_initializer.h>

    using namespace exotica;

    void run()
    {
        Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));

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
    ...
    
.. rubric:: CODE EXPLAINED

Initializer Headers
===================

At the top of the script, two initialiser header files are included: one
for the ``exotica_ik_solver/ik_solver_initializer.h`` and one for
``exotica_core_task_maps/eff_position_initializer.h``. These are generated from the ``.in`` files during the compilation/build phase.

When initializing manually, the appropriate initializers must be
included for both the task map and the solver. These are stored in the
``exotations`` directory under ``task_maps`` and ``solvers``
respectively.

Currently available solvers are:

.. code-block:: c++

    #include <exotica_ompl_solver/ompl_solver_initializer.h>
    #include <exotica_aico_solver/aico_solver_initializer.h>
    #include <exotica_ik_solver/ik_solver_initializer.h>

Once we have included the correct initializers, we must initialise:
* ``scene`` 
* ``map`` 
* ``problem`` 
* ``solver``

Scene Initialization
====================

When initializing the scene, we instantiate a ``SceneInitializer``, 
which here name ``scene``.

.. code-block:: c++

        // Scene using joint group 'arm'
        SceneInitializer scene("MyScene", "arm", false, "", "{exotica_examples}/resources/robots/lwr_simplified.urdf", "{exotica_examples}/resources/robots/lwr_simplified.srdf");

We must also pass in our initialization arguments seen in the `Scene Initializer <https://github.com/ipab-slmc/exotica/blob/master/exotica_core/init/scene.in>`__ file:

.. code-block:: c++

        class Scene

        extend <exotica_core/object>

        Required std::string JointGroup;

        Optional std::string RobotDescription = "robot_description";
        Optional std::string URDF = "";
        Optional std::string SRDF = "";
        Optional bool SetRobotDescriptionRosParams = false;  // to be used in conjunction with URDF or SRDF to set the robot_description and robot_description_semantic from the files/string in URDF/SRDF

        // CollisionScene
        Optional std::vector<exotica::Initializer> CollisionScene = std::vector<exotica::Initializer>();
        Optional bool AlwaysUpdateCollisionScene = false;      // Whether each Scene::Update triggers a CollisionScene::UpdateObjectTransforms()
        Optional bool DoNotInstantiateCollisionScene = false;  // If true, no CollisionScene plug-in will be loaded.

        // DynamicsSolver
        Optional std::vector<exotica::Initializer> DynamicsSolver = std::vector<exotica::Initializer>();

        Optional std::string LoadScene = "";  // to load multiple scenes, separate by semi-colon.
        Optional std::vector<exotica::Initializer> Links = std::vector<exotica::Initializer>();
        Optional std::vector<exotica::Initializer> Trajectories = std::vector<exotica::Initializer>();
        Optional std::vector<exotica::Initializer> AttachLinks = std::vector<exotica::Initializer>();

        // TODO: Move to CollisionScene
        Optional std::vector<std::string> RobotLinksToExcludeFromCollisionScene = std::vector<std::string>();
        Optional std::vector<std::string> WorldLinksToExcludeFromCollisionScene = std::vector<std::string>();



Here we use the parameters: 
* name of the scene ("MyScene") 
* name of the joint group ("arm") which is specified in the `SRDF <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/robots/lwr_simplified.srdf>`__ file.
* Debug argument ("false")
* RobotDescription ("")
* URDF (name of URDF file)
* SRDF (name of SRDF file)


Map Initialization
==================

Maps refers to the ``task maps`` of a problem, they provide a mapping from configuration space to task space
which are useful for fulfilling several tasks, such as specifying goals and avoiding obstacles. 
You can read more about task maps in a `later section <Task_maps.html>`__ . 

For now we are only interested in reaching an end effector goal, so we will use the ``EffFrame`` task map, 
which allows us specify the name of the end effector from the URDF file, which will be the focus when we 
try to reach a an end effector goal, as we are doing here. 

.. code-block:: xml

        EffFrameInitializer map("Position", false,
                                {FrameInitializer("lwr_arm_6_link", Eigen::VectorTransform(0, 0, 0, 0.7071067811865476, -4.3297802811774664e-17, 0.7071067811865475, 4.3297802811774664e-17))});

Here we create an EffFrameInitializer with the name "map". We again give the initialiser a name - "Position", which will be used to refer
to the map later. Then we give the standard debug argument (here it is false); then to initialise the frame we use the ``FrameInitializer``
initialiser to give the name of the end effector link (Must be the same name as the link in the URDF file). Then we can add an optional 
offset argument. 

*NOTE - the name of the end effector link must match that in the URDF
and SRDF files*

Problem Initialization
======================

In the steps up to this point, we have generated the components
which make up a problem. Now we can move onto initialising a problem
itself using these parts.

In this example we are interested in setting up a
``UnconstrainedEndPoseProblem``, so we use the
``UnconstrainedEndPoseProblemInitializer``. Naturally, if your problem
is a ``SamplingProblem``, then the ``SamplingProblemInitializer`` would
be used and so on. But here we have our current problem initialiser:

.. code-block:: c++

        UnconstrainedEndPoseProblemInitializer problem("MyProblem", scene, false, {map}, W);

into which we pass: 
* a name for the problem ``"MyProblem"`` (which we will use later) 
* the ``scene`` initialiser we created earlier (the name of the holder, not the name of the scene) 
* a debug argument. Here we set it to ``false`` 
* the ``map`` initialiser (must be contained in curly braces ``{}``)
* the weight vector ``W`` 

Later we will see in more detail that we can send multiple maps to the problem initialiser, all
contained within the curly braces e.g. ``{map,joint_limit_map,obs_avoid_map}`` with a map initialiser
for each of the variables inside the braces.

The ``W`` vector weights the joints of your robot according to the cost of moving each one. 
This vector must be the same size as the number of the number of DOF of your robot. 

Solver
======

That's the problem set up, now to do the same for the solver. For the
problem we have used in the tutorial (``UnconstrainedEndPoseProblem``),
the IK solver is the most appropriate solver, so this is the solver we
will set up:

.. code-block:: c++

        IKSolverInitializer solver("MySolver");
        solver.C = 1e-3;
        solver.MaxIterations = 1;
        solver.MaxStep = 0.1;

Again, we have an initialiser for the solver (``IKSolverInitializer``)
and we instantiate to a container, which here we call ``solver``. Also
during initialisation, we give the solver a name we we can refer to it
later; here we call the solver ``"MySolver"``

Solver Options
==============

After setting up the solver, there are some options to fill in to set
parameters for the solver, some required and some optional. Since we
have used the ``IKSolver`` in the tutorial, the options for this solver
are seen below:

.. code-block:: c++

        extend <exotica_core/motion_solver>
        Optional double Tolerance = 1e-5;
        Optional double Convergence = 0.0;
        Optional int MaxIterations = 50;
        Optional double MaxStep = 0.02;
        Optional double C = 0.0;
        Optional double Alpha = 1.0;

All selections in the ``IKSolver`` are optional. By referring back to
the example code, you see that we decided to set 3 of the options for
this solver:

.. code-block:: c++

        solver.C = 1e-3;
        solver.MaxIterations = 1;
        solver.MaxStep = 0.1;

This method is extensible to all the options in all the solvers. Before
initialising a solver, you should always look for initialisation
options, as some may be required. Again, these can be found in:

``exotica/exotations/solvers/<SolverName>/init/<SolverName>.in``

We now almost have a fully initialized motion solver. We can now
move onto the common initialization step between hard-coded and XML
initialization
`here <Common-Initialization-Step.html>`__.
