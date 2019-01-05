..  _eff_position:

End-effector position
=====================

End-effector position captures the translation of the relative frame transformation:

.. math::

    \Phi_\text{EffPos}(\boldsymbol{x}) = \boldsymbol{P}_A^B,

where :math:`\boldsymbol{P}_A^B` is translational part of :math:`\boldsymbol{M}_A^B`. The Jacobian of this task consists of the rows of the geometric Jacobian corresponding to the translation of the frame.