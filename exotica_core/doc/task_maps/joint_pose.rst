..  _joint_position:

Joint position
==============

Joint position task map computes the difference between the current joint configuration and a reference joint configuration:

.. math::

    \Phi_\text{Ref}(\boldsymbol{x}) = \boldsymbol{x}-\boldsymbol{x}_{\text{ref}},

where :math:`\boldsymbol{x}` is state vector of the joint configuration and :math:`\boldsymbol{x}_{\text{ref}}` is the reference configuration. The whole state vector :math:`x` may be used or a subset of joints may be selected. This feature is useful for constraining only some of the joints, e.g. constraining the back joints of a humanoid robot while performing a manipulation task. The Jacobian and Jacobian derivative are identity matrices. 

    We use notation :math:`x` for scalar values, :math:`\boldsymbol{x}` for vectors, :math:`X` for matrices, and :math:`\boldsymbol{X}` for vectorized matrices.
