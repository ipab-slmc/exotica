^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exotica_core_task_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.1.1 (2021-04-05)
------------------
* Fix unit tests on Debian Buster (`#734 <https://github.com/ipab-slmc/exotica/issues/734>`_)
* Contributors: Wolfgang Merkt

6.1.0 (2021-03-15)
------------------
* Add DistanceToLine2D task map (`#733 <https://github.com/ipab-slmc/exotica/issues/733>`_)
* Clang support and bug fixes (`#731 <https://github.com/ipab-slmc/exotica/issues/731>`_)
* Upgrade formatting to clang-format-6.0 (`#730 <https://github.com/ipab-slmc/exotica/issues/730>`_)
* Contributors: Wolfgang Merkt

6.0.2 (2020-11-23)
------------------

6.0.1 (2020-11-17)
------------------

6.0.0 (2020-11-08)
------------------
* Remove usage of exotica_collision_scene_fcl; add backwards compatibility
* Fix segmentation fault for joint smoothing task maps
* test_maps: Upgrade collision scene to initializer
* EffFrame: Fix indexing bug for >1 end-effector
* test_maps: Add multi-end-effector unit test for EffFrame
* JointPose: Add set_joint_ref
* test_maps: testCollisionDistance - update initialiser, deactivate Jacobian test
* test_maps: Change to lp-Infinity-norm
* CollisionDistance: Fix Jacobian being off by -1
* test_maps: Change test_jacobian to central difference
* test_maps: Add collision links to the URDF
* Fix unittests for older versions of googletest
* Do not re-define GoogleTest color constants
* More informative exceptions
* package.xml: Add missing test dependencies
* CMakeLists: Upgrade minimum version to 3.0.2 to avoid CMP0048
* Add note to test_maps that SphereCollision throws
* SmoothCollisionDistance: Fix in point Jacobian
* Add VariableSizeCollisionDistance
* SmoothCollisionDistance: Support nullptr for collision elements
* SmoothCollisionDistance: Make more readable, allocate less in loop
* JointPose: Implement get_joint_ref, get_joint_map
* EffAxisAlignment: Efficiency improvements
* Distance: Implement Hessian
* Add README tracking Hessian implementations
* test_maps: Test ContinuousJointPose Hessian
* PointToPlane: Implement Hessian, Simplify equation to EffPositionZ
* Test JointLimit, expose ControlRegularization to Python
* JointPose: Implement Hessian
* EffFrame: Implement Hessian
* test_maps: Test hessians for EffPosition, EffPositionXY, EffOrientation
* EffOrientation: Implement Hessian
* EffPositionXY: Implement Hessian
* test_maps: More informative exception handling
* test_maps: Set default DerivativeOrder to 2
* test_maps: Add test_hessian
* test_maps: Change test_jacobian to central difference
* Add ControlRegularization
* EffPosition: Fix Hessian indexing
* Make rotation_type private, add getter, fix instantiation
* EffPosition: Implement Hessian
* JointLimit: Fix Jacobian, add dynamic update methods
* Fix unit test for testJointLimit
* ContinuousJointPose: Add Hessian
* test_maps: Use const-ref
* SumOfPenetrations: Update to new, faster collision distance
* EffAxisAlignment: Clear debug topic on start
* EffAxisAlignment: Added debug visualisation
* Add unittest for EffPositionXY
* Contributors: Wolfgang Merkt

5.1.3 (2020-02-13)
------------------
* Refactor CollisionScene, add faster distance checks, speedup SmoothCollisionDistance (`#688 <https://github.com/ipab-slmc/exotica/issues/688>`_)
* Contributors: Wolfgang Merkt

5.1.2 (2020-02-10)
------------------

5.1.1 (2020-02-10)
------------------

5.1.0 (2020-01-31)
------------------
* Added various new taskmaps
* Contributors: Chris Mower, Christian Rauch, Vladimir Ivan, Wolfgang Merkt
