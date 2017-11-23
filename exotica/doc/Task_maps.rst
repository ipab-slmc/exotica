# Task Maps

Task maps form the basis of all Problems in EXOTica; they provide a mapping from configuration space to task space. A problem can contain one or many task maps. For example, if the problem requires the effector to reach a goal position with no concern for orientation we might use the end-effector position task map: `EffPosition`, which informs EXOTIca that this is what you require. Whereas if we are only concerned with orientation, we would use the 'EffOrientation' map. If we care about both position and orientation we can send BOTH the 'EffPosition' AND the 'EffOrientation' task maps to the problem or we can use the 'EffFrame' task map, which contains both positional and orientation mapping. 

The ability to specify multiple task maps enables a wide range of customization for each problem. As well as being able to specify what frame and joint is of interest, we can add collision detection arguments, joint limit constraints or centre of mass location among others individually or collectively to problems.

In this tutorial we demonstrate initialisation of task maps and list the different types and uses of task map.

## Initialising task maps 

 	CoM: Specify the CoM for a link
 	Initialisation:  
 	.. code:: c++
 		(std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), bool EnableZ_ =  true)


	Distance: Returns the distance between two frames, either an end effector frame or a point frame.

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>())



	EffOrientation: specifies an end effector orientation in task space

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string 	Type_ =  "RPY")


	EffPosition: specifies an end effector position in task space

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>())

	EffFrame: specifies an end effector pose in task space

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string Type_ =  "RPY")


	IMesh: See http://homepages.inf.ed.ac.uk/svijayak/publications/ivan-IJRR2013.pdf for details about iMesh

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string ReferenceFrame_ =  "/world", Eigen::VectorXd Weights_ =  Eigen::VectorXd())

	Identity: the position of a joint. Useful if you want to avoid a certain position. 

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), Eigen::VectorXd JointRef_={}, std::vector<int> JointMap_={})

	JointLimit: map to keep joints away from limits. Use options to set penalties for nearing joint limits

	Initialisation: 
	.. code:: c++
	( std::string Name_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), double SafePercentage_ =  0.0, std::string RobotDescription_ =  "robot_description")

	Sphere: Initiates a sphere object in relation to a named link with an offset - acts as a primitive for collision SphereCollision

	Initialisation: 
	.. code:: c++
	( std::string Link_, double Radius_, Eigen::VectorXd LinkOffset_ =  Eigen::IdentityTransform(), std::string Base_ =  "", Eigen::VectorXd BaseOffset_ =  Eigen::IdentityTransform(), std::string Group_ =  "default")

	SphereCollision: Used in collision detection. Groups of spheres (seen in previous bullet point) are attached to the robot and environment. Spheres within the same group will not detect collisions within the same group, but collisions between 

	Initialisation: 
	.. code:: c++
	( std::string Name_, double Precision_, bool Debug_ =  false, std::vector<exotica::Initializer> EndEffector_ =  std::vector<exotica::Initializer>(), std::string ReferenceFrame_ =  "/world", double Alpha_ =  1.0)
