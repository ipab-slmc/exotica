**************************
Common Initialization Step
**************************

This is the final step of initialization before we can use EXOTica to
solve motion plans.

In this section we will be using the containers we set up in the previous
initialization tutorials, so if you have not completed either:
`XML <XML.html>`__ (both initialization and parsing steps) or `manual
initialization <manual_initialization.html>`__, please complete those first.

If you have done this, the robot properties have been set up, either through
XML or within your C++ or Python code. The completed initializers now need to used to create the problem/solver objects and the problem specified to the solver.

In both our XML and manually coded initializers, we created a problem
initializer in the ``problem`` variable and the solver in the ``solver``
variable. These now need to be sent to generic problem and solver holders:

.. code-block:: c++

        //... continued from XML or hardcoded initialization

        // Initialize

        PlanningProblemPtr any_problem = Setup::CreateProblem(problem);
        MotionSolverPtr any_solver = Setup::CreateSolver(solver);

        // Assign the problem to the solver
        any_solver->SpecifyProblem(any_problem);
        UnconstrainedEndPoseProblemPtr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

        ....

Sending Problems to Solvers
===========================

Once we have the problem in a generic holder pointer (``any_problem``),
we can send it to the solver (``any_solver``). Internally, this assigns a pointer to the problem within the solver such that the solver can use/solve the problem (by repeatedly updating the problem and evaluating cost/constraints etc.).

.. code-block:: c++

        any_solver->SpecifyProblem(any_problem);

Problem Pointer Setup
=====================

In order to call problem-specific methods, we need to create a problem-specific holder from the generic pointer to the ``PlanningProblem`` -- this will allow us to access the specific properties of these
problems later on:

.. code-block:: c++

        UnconstrainedEndPoseProblemPtr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

After these steps, EXOTica is fully initialized. We can move on to using
EXOTica's functionality in the `next tutorial <using_exotica_cpp.html>`__.
