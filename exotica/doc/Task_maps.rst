**********
Task Maps
**********

Task maps form the basis of all Problems in EXOTica; they provide a mapping from configuration space to task space. A problem can contain one or many task maps. 
For example, if the problem requires the effector to reach a goal position with no concern for orientation we might use the end-effector position task map: 
`EffPosition`. Whereas if we are only concerned with orientation, we would use the 'EffOrientation' map. If we care about both position and orientation we can 
send BOTH the 'EffPosition' AND the 'EffOrientation' task maps to the problem or we can use the 'EffFrame' task map, which contains both positional and orientation mapping. 

The ability to specify multiple task maps enables a wide range of customization for each problem. As well as being able to specify what frame and joint is of interest, 
we can add collision detection arguments, joint limit constraints or centre of mass location among others individually or collectively to problems.

In this tutorial we explain each of the task maps and detail the input arguments needed to instantiate the task maps.

Initializing task maps
======================

Each task map needs to be initialized first, this sets up the task map and specifies any important parameters. After initialization we can send the the task map to 
the problem. 

.._initializing-com-task-map:
CoM 
---

Specify the CoM for a link

Initialization:  

.. code-block:: cpp
	
    (std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), bool EnableZ_ =  true)


Distance
--------

Returns the distance between two frames, either an end effector frame or a point frame.

Initialization:

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>())



EffOrientation
--------------

Specifies an end effector orientation in task space

Initialization: 

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string 	Type_ =  "RPY")


EffPosition
-----------

Specifies an end effector position in task space

Initialization: 

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>())

EffFrame
--------

Specifies an end effector pose in task space

Initialization: 

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string Type_ =  "RPY")


InteractionMesh
---------------

See http://homepages.inf.ed.ac.uk/svijayak/publications/ivan-IJRR2013.pdf for details about iMesh

Initialization: 

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string ReferenceFrame_ =  "/world", Eigen::VectorXd Weights_ =  Eigen::VectorXd())

JointPose
---------

The position of a joint. Useful if you want to avoid a certain position. 

Initialization: 

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), Eigen::VectorXd JointRef_={}, std::vector<int> JointMap_={})

JointLimit
----------

Map to keep joints away from limits. Use options to set penalties for nearing joint limits

Initialization: 

.. code-block:: cpp

    ( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), double SafePercentage_ =  0.0, std::string RobotDescription_ =  "robot_description")

Sphere
------

Initiates a sphere object in relation to a named link with an offset - acts as a primitive for collision SphereCollision

Initialization: 

.. code-block:: cpp

    ( std::string Link_, double Radius_, Eigen::VectorXd LinkOffset_ =  Eigen::IdentityTransform(), std::string Base_ =  "", Eigen::VectorXd BaseOffset_ =  Eigen::IdentityTransform(), std::string Group_ =  "default")

SphereCollision
---------------

Used in collision detection. Groups of spheres (seen in previous bullet point) are attached to the robot and environment. 
Spheres within the same group will not detect collisions within each other , but collisions between different groups are detected.

Initialization: 

.. code-block:: cpp

    ( std::string Name_, double Precision_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string ReferenceFrame_ =  "/world", double Alpha_ =  1.0)


Using Task Maps
===============

Once we've chosen and initialized the task maps we're interested in they need to be sent to the problem. The problem then informs the solver that these things must be taken into
consideration when producing a motion plan. Each problem can handle one of many task maps. Let's look at how we send these to the problem. 

.._using-task-maps-cpp:
C++
---

In the snippet below, we see that we have created a task map named ``map`` , which is an end effector frame map. We send this to the problem in the appropriate argument place using 
the curly brackets ``{map}``. In this snippet we assume you have already created a ``scene`` and ``W`` initializer:

.. code-block:: cpp

    EffFrameInitializer map("Position", false,
                            {FrameInitializer("lwr_arm_6_link", Eigen::VectorTransform(0, 0, 0, 0.7071067811865476, -4.3297802811774664e-17, 0.7071067811865475, 4.3297802811774664e-17))});

    UnconstrainedEndPoseProblemInitializer problem("MyProblem", scene, false, {map}, W);

By placing multiple map variable names inside these curly brackets, we can specify several maps to be sent to the same problem (this sets joint limits for the 
`LWR_simplified <https://github.com/ipab-slmc/exotica/blob/master/exotica_examples/resources/robots/lwr_simplified.urdf>`__  arm in the examples):

.. code-block:: cpp

    void get_joint_limits(std::vector<Initializer> joint_store)
    {
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_0_link")})});
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_1_link")})});
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_2_link")})});
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_3_link")})});
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_4_link")})});
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_5_link")})});
        joint_store.push_back({EffFrameInitializer("Position",false,{FrameInitializer("lwr_arm_6_link")})});
    }

    std::vector<Initializer> joint_store;
        get_joint_limits(joint_store);

    JointLimitInitializer joint_map("joint_limits",false,joint_store,90.0);

    EffFrameInitializer eff_map("Position", false,
                            {FrameInitializer("lwr_arm_6_link", Eigen::VectorTransform(0, 0, 0, 0.7071067811865476, -4.3297802811774664e-17, 0.7071067811865475, 4.3297802811774664e-17))});

    UnconstrainedEndPoseProblemInitializer problem("MyProblem", scene, false, {eff_map,joint_map}, W);

Further task maps can then be added in the same way. These can now be sent to the solver. 