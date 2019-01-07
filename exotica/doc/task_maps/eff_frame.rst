..  _eff_frame:

End-effector frame
==================

End-effector frame task map captures the relative transformation between the base frame :math:`B` and the tip frame :math:`A`:

.. math::

    \Phi_\text{EffFrame}(\boldsymbol{x}) = \boldsymbol{M}_A^B,

where :math:`\boldsymbol{M}_A^B\in SE(3)` is computed using the system model using the ``Scene``. We use the ``task space vector`` data structure (described later in this section) to handle storage and operations on spatial frames. The Jacobian of this task map is the geometric Jacobian computed by the ``Scene``.