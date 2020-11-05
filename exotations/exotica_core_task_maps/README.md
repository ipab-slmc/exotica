# exotica_core_task_maps

This package contains a selection of commonly used TaskMaps. These are either functions of configuration phi(q) or functions of the full state and control phi(x,u). They also implement partial derivatives (Jacobian / Hessian).

## TaskMaps that support analytical Hessians

  - ContinuousJointPose
  - ControlRegularization
  - Distance
  - EffFrame
  - EffOrientation
  - EffPosition
  - EffPositionXY
  - JointLimit
  - JointPose
  - PointToPlane => Can also be interpreted as "EffPositionZ"

## TaskMap that do not yet have analytical Hessians (and thus default to Gauss-Newton approximation)

  - [ ] AvoidLookAtSphere
  - [ ] CenterOfMass => also the first link ("base") mass is currently ignored :'(
  - [ ] CollisionCheck
  - [ ] CollisionDistance
  - [ ] EffAxisAlignment
  - [ ] EffBox
  - [ ] EffVelocity
  - [ ] GazeAtConstraint
  - [ ] InteractionMesh
  - [ ] JointAccelerationBackwardDifference
  - [ ] JointJerkBackwardDifference
  - [ ] JointTorqueMinimizationProxy
  - [ ] JointVelocityBackwardDifference
  - [ ] JointVelocityLimit
  - [ ] JointVelocityLimitConstraint
  - [ ] Manipulability
  - [ ] PointToLine
  - [ ] QuasiStatic
  - [ ] SmoothCollisionDistance
  - [ ] SphereCollision
  - [ ] SumOfPenetrations

## TaskMaps known to have discontinuous gradients for NLPs

*Time-indexed task maps* 

  - [ ] EffVelocity
  - [ ] JointVelocityLimit
