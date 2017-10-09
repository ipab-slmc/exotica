*********************
Manual Initialisation
*********************

Manual initialisation encapsulates all initialisation and handling of
EXOTica within your own C++ code, with no external XML files. This could
be a preferred if your project has been finalised and does not need to
be changed or if you prefer to have all your EXOTica code in one place.

In this tutorial, we will use the example of manual initialisation for
the UnconstrainedEndPoseProblem found in the ``exotica_examples``
`here <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/src/manual.cpp>`__:

.. code:: c++

    #include <exotica/Exotica.h>

    // Manual initialization requires dependency on specific solvers and task maps:
    #include <ik_solver/IKsolverInitializer.h>
    #include <task_map/EffPositionInitializer.h>

    using namespace exotica;

    void run()
    {
        // Scene using joint group 'arm'
        SceneInitializer scene("MyScene","arm");

        // End-effector task map with two position frames
        EffPositionInitializer map("Position","MyScene",false,
        {FrameInitializer("lwr_arm_6_link")});

        // Create a task using the map above (goal will be specified later)
        Eigen::VectorXd W(7);
        W << 7,6,5,4,3,2,1;

        UnconstrainedEndPoseProblemInitializer problem("MyProblem",scene,W,false,{map});
        IKsolverInitializer solver("MySolver");
        solver.C = 1e-3;
        solver.MaxIt = 1;
        solver.MaxStep = 0.1;
    ...

The Code Explained
------------------

Include Initialisers
~~~~~~~~~~~~~~~~~~~~

At the top of the script, two initialiser header files are included: one
for the ``ik_solver/IKsolverInitializer.h`` and one for
``task_map/EffPositionInitializer.h``.

When initialising manually, the appropriate initialisers must be
included for both the task map and the solver. These are stored in the
``exotations`` directory under ``task_maps`` and ``solvers``
respectively.

Currently available solvers are:

.. code:: c++

    #include <ompl_solver/OMPLsolverInitializer.h>
    #include <aico/AICOsolverInitializer.h>
    #include <ik_solver/IKsolverInitializer.h>

Once we have included the correct initialisers, we must initialise the:
\* ``scene`` \* ``map`` \* ``task`` \* ``problem`` \* ``solver``

Scene Initialisation
~~~~~~~~~~~~~~~~~~~~

When initialising the scene, we create an instance of the
``SceneInitializer``, which here we assign the name ``scene``.

.. code:: c++

        // Scene using joint group 'arm'
        SceneInitializer scene("MyScene","arm");

We must also pass in two arguments during initialisation: the name of
the scene (to be used later) and the name of the joint group specified
in the SRDF file.
`Here <https://github.com/openhumanoids/exotica/blob/master/examples/exotica_examples/resources/lwr_simplified.srdf>`__
in the example SRDF we see:

.. code:: xml

    <group name="arm">

The name given in this file must match the one that is passed into the
scene initialiser. This will direct EXOTica to the dimensions of the arm
used for motion planning.

Map Initialisation
~~~~~~~~~~~~~~~~~~

Maps are initialised in the same way as scenes. Here we name the
``EffPositionInitializer`` ``map`` and a we call the map by the name
``"Position"``.

.. code:: c++

        // End-effector task map with a position frame
        EffPositionInitializer map("Position","MyScene",false,
        {FrameInitializer("lwr_arm_6_link")});

From here, the map initialiser also requires a ``scene``, a debug
argument and an end-effector link.

We provide the ``scene`` to the map using the name of the scene we
created in the previous block. In this tutorial we used the name
``"MyScene"``, so this is what we will pass to the ``map``.

Next, we'll set the ``debug argument``; this can be true or false
depending on your preference.

Finally, we must set the end effector link by wrapping it within a
``FrameInitializer``. The name of the end effector that is supplied must
be the name of a link within the URDF file.

When we plug in the name of the end-effector ``"lwr_arm_6_link"`` from
our example URDF file, we have:

.. code:: c++

    {FrameInitializer("lwr_arm_6_link")}

Task
~~~~

*This needs to be rewritten* To create a task, we must assign a weight
to each DOF to denote its relative movement cost. This is done by
providing a vector of weights ``W`` which tends to be filled in
descending order based on the DOF:

.. code:: c++

        Eigen::VectorXd W(7);
        W << 7,6,5,4,3,2,1;

As an example, if we had a 3 DOF robot, the weights ``W`` would be set
at:

.. code:: c++

        Eigen::VectorXd W(3);
        W << 3,2,1;

This weight vector will be sent to the problem in the next step.

Problem
~~~~~~~

In the steps up to this point, we have created generated the components
which make up a problem. Now we can move onto initialising a problem
using these parts.

In this example we are interested in setting up a
``UnconstrainedEndPoseProblem``, so we use the
``UnconstrainedEndPoseProblemInitializer``. Naturally, if your problem
is a ``SamplingProblem``, then the ``SamplingProblemInitializer`` would
be used and so on. But here we have our current problem initialiser:

.. code:: c++

        UnconstrainedEndPoseProblemInitializer problem("MyProblem",scene,W,false,{map});

into which we pass: \* a name for the problem ``"MyProblem"`` (which we
will use later) \* the ``scene`` initialiser we created earlier (the not
simply the name of the scene) \* the weight vector ``W`` \* a debug
argument. Here we set it to ``false`` \* the ``map`` initialiser (must
be contained in curly braces ``{}``)

Solver
~~~~~~

That's the problem set up, now to do the same for the solver. For the
problem we have used in the tutorial (``UnconstrainedEndPoseProblem``),
the IK solver is the most appropriate solver, so that is the solver we
will set up:

.. code:: c++

        IKsolverInitializer solver("MySolver");
        solver.C = 1e-3;
        solver.MaxIt = 1;
        solver.MaxStep = 0.1;

Again, we have an initialiser for the solver (``IKsolverInitializer``)
and we instantiate to a container, which here we call ``solver``. Also
during initialisation, we give the solver a name we we can refer to it
later; here we call the solver ``"MySolver"``

Solver Options
^^^^^^^^^^^^^^

After setting up the solver, there are some options to fill in to set
parameters for the solver, some required and some optional. Since we
have used the ``IKSolver`` in the tutorial, the options for this solver
are seen below:

.. code:: xml

    extend <exotica/MotionSolver>
    Optional double Tolerance = 1e-5;
    Optional double Convergence = 0.0;
    Optional int MaxIt = 50;
    Optional double MaxStep = 0.02;
    Optional double C = 0.0;
    Optional double Alpha = 1.0;

All selections in the ``IKSolver`` are optional. By referring back to
the example code, you see that we decided to set 3 of the options for
this solver:

.. code:: c++

        solver.C = 1e-3;
        solver.MaxIt = 1;
        solver.MaxStep = 0.1;

This method is extensible to all the options in all the solvers. Before
initialising a solver, you should always look for initialisation
options, as some may be required. Again, these can be found in:

``exotica/exotations/solvers/<SolverName>/init/<SolverName>.in``

We now almost have a fully initialised hard-coded script. We can now
move onto the common initialisation step between hard-coded and XML
initialisation
`here <https://github.com/openhumanoids/exotica/wiki/Common-Initialisation-Step>`__.
