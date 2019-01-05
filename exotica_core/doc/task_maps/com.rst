..  _com:

Center of mass
==============

Center of mass task map computes the center-of-mass of all of the robot links defined in the system model:

.. math::

    \Phi_\text{CoM}(\boldsymbol{x}) = \sum_i(\boldsymbol{P}_{\text{CoM}_i}^\text{world}m_i),

where :math:`\boldsymbol{P}_{\text{CoM}_i}^\text{world}` is the position of the center-of-mass of the :math:`i`-th link w.r.t. the world frame, and :math:`m_i` is mass of the :math:`i`-th body. The Jacobian is computed using the chain rule. This task map can also be initialized to compute the projection of the center-of-mass on the :math:`xy`-plane. In this case, the :math:`z`-component is removed.