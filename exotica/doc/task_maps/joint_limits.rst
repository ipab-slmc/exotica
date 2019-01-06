..  _joint_limits:

Joint limits
============

Joint limits task map assigns a cost for violating joint limits. The joint limits are loaded from the robot model. The mapping is calculated as:

.. math::

    \Phi_\text{Bound}(x) = 
    \begin{cases}
    x - x_\text{min} - \epsilon, & \text{if } x < x_\text{min}+\epsilon \\
    x - x_\text{max} + \epsilon, & \text{if } x > x_\text{max}-\epsilon \\
    0,                       & \text{otherwise}
    \end{cases},

where :math:`x_\text{min}` and :math:`x_\text{max}` are lower and upper joint limits respectively, and :math:`\epsilon\geq0` is a safety margin. The Jacobian and Jacobian derivative are identity matrices. 