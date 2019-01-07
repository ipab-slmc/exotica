**************************
Common Initialization Step
**************************

This is the final step of initialization before we can use EXOTica to
solve motion plans.

In this section we will be using containers we set up in the previous
initialization tutorials, so if you have not completed either:
`XML <XML.html>`__ (both initialization and parsing steps) or `manual
initialisation <Manual-Initialisation.html>`__, then go back and do those first.

If you have done this, the robot properties have been set-up, either through
XML or within your C++ or Python code. The completed initializers now need to be sent to the
appropriate places and the problem sent to the solver.

In both our XML and manually coded initializers, we created a problem
initializer in the 'problem' variable and the solver in the 'solver'
variable. These now need to be sent to generic problem and solver holders:

.. code-block:: c++

        //... continued from XML or hardcoded initialization

        // Initialize

        PlanningProblem_ptr any_problem = Setup::createProblem(problem);
        MotionSolver_ptr any_solver = Setup::createSolver(solver);

        // Assign the problem to the solver
        any_solver->specifyProblem(any_problem);
        UnconstrainedEndPoseProblem_ptr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

        ....

Sending Problems to Solvers
===========================

Once we have the problem in a generic holder pointer ("any\_problem"),
we can send it to the solver ("any\_solver").

.. code-block:: c++

        any_solver->specifyProblem(any_problem);

Problem Pointer Setup
=====================

Then use the generic problem holder to create a specific problem for our
purposes - this will allow us to access the specific properties of these
problems later on:

.. code-block:: c++

        UnconstrainedEndPoseProblem_ptr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

After these steps, EXOTica is fully initialized. We can move on to using
EXOTica's functionality in the `next
tutorial <Using-EXOTica.html>`__.
