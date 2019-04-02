..  _unconstrained_time_indexed_problem:

Unconstrained time-indexed problem
==================================

The unconstrained time-indexed problem (``UnconstrainedTimeIndexedProblem``) defines a problem minimizing the quadratic cost over a trajectory using a linearized model of the system. The trajectory is uniformly discretized in time. The state is represented by the combined vector of joint configurations for each time step. The problem is formulated as:

.. math::

    \underset{\boldsymbol{x}}{\text{argmin}} (f(\boldsymbol{x})^\top Q f(\boldsymbol{x})).

The time step duration :math:`\vartriangle\!\!t` and number of time steps :math:`T` are specified. Additionally, configuration space weighting :math:`W` is specified. This allows us to scale the cost of moving each joint (or control variable) individually (analogous to the weighting :math:`W` used in the :ref:`unconstrained_end_pose_problem`). The cost function is then defined as:

.. math::

    f(\boldsymbol{x})=\sum_t \sum_i \rho_{i,t}||\Phi_{i,t}(\boldsymbol{x})-\boldsymbol{y}^*_{i,t}||,

where :math:`t\in(1, ..., T)` is the time index.

.. todo::

    Transition vs task cost
