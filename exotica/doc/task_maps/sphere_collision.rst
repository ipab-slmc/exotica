..  _sphere_collision:

Sphere collision
=================

Sphere collision task map provides a differentiable collision distance metric. The collision shapes are approximated by spheres. Each sphere is attached to the kinematic structure of the robot or to the environment. Each sphere is then assigned a collision group, e.g. :math:`i\in \mathcal{G}`. Spheres within the same group do not collide with each other, while spheres from different groups do. The collision cost is computed as:

.. math::

    \Phi_\text{CSphere}(\boldsymbol{x}) = \sum_{i,j}^{G}\frac{1}{1+ e^{5\epsilon (||\boldsymbol{P}_i^\text{world}-\boldsymbol{P}_j^\text{world}||-r_i-r_j)} }, 

where :math:`i, j` are indices of spheres from different collision groups, :math:`\epsilon` is a precision parameter, :math:`\boldsymbol{P}_i^\text{world}` and :math:`\boldsymbol{P}_j^\text{world}` are positions of the centers of the spheres, and :math:`r_i, r_j` are the radii of the spheres. The sigmoid function raises from :math:`0` to :math:`1`, with the steepest slope at the point where the two spheres collide. Far objects contribute small amount of error while colliding objects produce relatively large amounts of error. The precision parameter can be used to adjust the fall-off of the error function, e.g. a precision factor of :math:`10^3` will result in negligible error when the spheres are further than :math:`10^{-3}` m apart. The constant multiplier of :math:`5` in was chosen to achieve this fall-off profile.