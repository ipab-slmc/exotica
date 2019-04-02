..  _sampling_problem:


Sampling problem
================

The sampling problem (``SamplingProblem``) defines a class of kinematic problems that do not require any cost function. Each state is evaluated for validity but not for quality. 
This is formalised as:

.. math::

    \underset{\boldsymbol{x}}{\arg}~f(\boldsymbol{x})=\text{True}

We refer to a problem with binary variables as a sampling problem because randomized or another type of sampling is required to solve them. These types of problems often cannot be easily solved by numerical optimization because their constraints are not differentiable.

In EXOTica, we support both equality (task maps that evaluate to zero for validity) and inequality (task maps that have to evaluate to less than zero for validity) constraints for sampling problems. As sampling on constraint manifolds is a problem, we support setting tolerances for equality and inequality constraints in sampling problems.

This type of problem also requires a goal configuration :math:`\boldsymbol{x}^*`. The objective of the solvers is to compute a valid (feasible) trajectory from the start state to the goal state. The validity of each state (and the itermediate states) is checked by applying a threshold :math:`\epsilon` on the output of the task map: :math:`\rho_i(\Phi_i(\boldsymbol{x})-\boldsymbol{y}^*_i)<\epsilon`. The output trajectory is not indexed on time and the number of configurations may vary between solutions. This type of planning problem is used with sampling-based motion solvers.



