..  _eff_orientaiton:

End-effector orientation
========================

End-effector orientation captures the rotation of the relative frame transformation:

.. math::

    \Phi_\text{EffRot}(\boldsymbol{x}) = \boldsymbol{R}_A^B,

where :math:`\boldsymbol{R}_A^B\in SO(3)` is rotational part of :math:`\boldsymbol{M}_A^B`. Similarly to the ``end-effector frame`` task map, the storage and the operations on the resulting :math:`SO(3)` space are implemented within the ``task space vector``. The Jacobian of this task consists of the rows of the geometric Jacobian corresponding to the rotation of the frame.