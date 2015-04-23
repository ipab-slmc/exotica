#include "kinematica/KinematicTree.h"

#ifdef KIN_DEBUG_MODE
#include <iostream>
#endif

#ifdef EXOTICA_DEBUG_MODE
#define CHECK_EXECUTION         std::cout << "Ok in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n"; //!< With endline
#define INDICATE_FAILURE        std::cerr << "Failed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n";//!< With endline
#define WARNING(x)							 std::clog << "Warning in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ": " << x << "\n";//!< With endline
#define ERROR(x)								 std::cerr << "Failed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\n";//!< With endline
#define INFO(x)									 std::clog << "Info in " << __PRETTY_FUNCTION__ << ": " << x << "\n";//!< With endline
#else
#define CHECK_EXECUTION   //!< No operation
#define INDICATE_FAILURE        std::cerr << "Failed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n";//!< With endline
#define WARNING(x)        //!< No operation
#define ERROR(x)								 std::cerr << "Failed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\n";//!< With endline
#define INFO(x)
#endif

int kinematica::KinematicTree::getNumJoints()
{
	return num_jnts_spec_;
}

kinematica::KinematicTree::KinematicTree()
{
#ifdef KIN_DEBUG_MODE
	std::cout << "Default Constructor ... ";
#endif

	//!< Set to default values
	zero_undef_jnts_ = false;

#ifdef KIN_DEBUG_MODE
	std::cout << "Done" << std::endl;
#endif
}

kinematica::KinematicTree::KinematicTree(const std::string & urdf_param,
		const SolutionForm_t & optimisation)
{
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialiser Constructor ... (urdf-file)";
#endif
	//!< Set to default values
	zero_undef_jnts_ = false;

	//!< Attempt initialisation
	initKinematics(urdf_param, optimisation);

#ifdef KIN_DEBUG_MODE
	std::cout << "Done" << std::endl;
#endif
}

kinematica::KinematicTree::KinematicTree(const KDL::Tree & temp_tree,
		const SolutionForm_t & optimisation)
{
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialiser Constructor ... (temp-tree)";
#endif
	//!< Set to default values
	zero_undef_jnts_ = false;

	//!< Attempt initialisation
	initKinematics(temp_tree, optimisation);

#ifdef KIN_DEBUG_MODE
	std::cout << "Done" << std::endl;
#endif
}

kinematica::KinematicTree::KinematicTree(const kinematica::KinematicTree & rhs)
{
	robot_tree_ = rhs.robot_tree_;
	segment_map_ = rhs.segment_map_;
	used_joints_ = rhs.used_joints_;
	used_joints_segs_ = rhs.used_joints_segs_;
	zero_undef_jnts_ = rhs.zero_undef_jnts_;
	num_jnts_spec_ = rhs.num_jnts_spec_;
	eff_segments_ = rhs.eff_segments_;
	eff_seg_offs_ = rhs.eff_seg_offs_;
	eff_segments_ini_ = rhs.eff_segments_ini_;
	eff_seg_offs_ini_ = rhs.eff_seg_offs_ini_;

	forward_map_ = rhs.forward_map_;
	jacobian_ = rhs.jacobian_;
	com_ = rhs.com_;
}

kinematica::KinematicTree & kinematica::KinematicTree::operator=(
		const kinematica::KinematicTree & rhs)
{
	robot_tree_ = rhs.robot_tree_;
	segment_map_ = rhs.segment_map_;
	used_joints_ = rhs.used_joints_;
	used_joints_segs_ = rhs.used_joints_segs_;
	zero_undef_jnts_ = rhs.zero_undef_jnts_;
	num_jnts_spec_ = rhs.num_jnts_spec_;
	eff_segments_ = rhs.eff_segments_;
	eff_seg_offs_ = rhs.eff_seg_offs_;
	eff_segments_ini_ = rhs.eff_segments_ini_;
	eff_seg_offs_ini_ = rhs.eff_seg_offs_ini_;
	forward_map_ = rhs.forward_map_;
	jacobian_ = rhs.jacobian_;
	com_ = rhs.com_;
	return *this;
}

bool kinematica::KinematicTree::initKinematics(const std::string & urdf_param,
		const kinematica::SolutionForm_t & optimisation)
{
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialisation Function ... (File-variant) ... " << std::endl;
#endif
	//!< Local Declarations & Lock
	KDL::Tree temp_tree;  //!< KDL Tree structure from urdf
	boost::mutex::scoped_lock(member_lock_);
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialisation Function ... Locked" << std::endl;
#endif
	if (kdl_parser::treeFromParam(urdf_param, temp_tree))
	{
		return initialise(temp_tree, optimisation);
	}
	else
	{
		return false;
	}
}

bool kinematica::KinematicTree::initKinematics(const KDL::Tree & temp_tree,
		const kinematica::SolutionForm_t & optimisation)
{
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialisation Function ... (Tree-variant) ... " << std::endl;
#endif
	//!< Lock
	boost::mutex::scoped_lock(member_lock_);
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialisation Function ... Locked" << std::endl;
#endif
	return initialise(temp_tree, optimisation); //!< Just call
}

bool kinematica::KinematicTree::initKinematics(tinyxml2::XMLHandle & handle)
{
#ifdef KIN_DEBUG_MODE
	std::cout << "Initialisation from xml" << std::endl;
#endif
	std::string urdf_file;
	kinematica::SolutionForm_t solution;

	//!< Checks for compulsaries
	if (!handle.FirstChildElement("Urdf").ToElement())
	{
#ifdef KIN_DEBUG_MODE
		std::cout<<"Urdf element not exist"<<std::endl;
#endif
		return false;
	} //!< We must have a urdf specification at the very least
	if (!handle.FirstChildElement("Update").ToElement())
	{
#ifdef KIN_DEBUG_MODE
		std::cout<<"Update element not exist"<<std::endl;
#endif
		return false;
	} //!< We must have the list of joints

//!< First the file
	urdf_file = handle.FirstChildElement("Urdf").ToElement()->GetText();
	if (urdf_file.empty())
	{
#ifdef KIN_DEBUG_MODE
		std::cout<<"URDF is empty"<<std::endl;
#endif
		return false;
	}

//!< Now the solution params:
	solution.root_segment = "";
	solution.root_seg_off = KDL::Frame::Identity();
	if (handle.FirstChildElement("Root").ToElement())
	{
		if (!handle.FirstChildElement("Root").ToElement()->Attribute("segment"))
		{
#ifdef KIN_DEBUG_MODE
			std::cout<<"Root element not exist"<<std::endl;
#endif
			return false;
		}
		solution.root_segment = handle.FirstChildElement("Root").ToElement()->Attribute("segment");

		if (handle.FirstChildElement("Root").FirstChildElement("vector").ToElement())
		{
			Eigen::VectorXd temp_vector;
			if (!xmlGetVector(*(handle.FirstChildElement("Root").FirstChildElement("vector").ToElement()), temp_vector))
			{
#ifdef KIN_DEBUG_MODE
				std::cout<<"Get root position vector failed"<<std::endl;
#endif
				return false;
			}
			if (temp_vector.size() != 3)
			{
#ifdef KIN_DEBUG_MODE
				std::cout<<"Root position vector size is invalid"<<std::endl;
#endif
				return false;
			}
			solution.root_seg_off.p.x(temp_vector(0));
			solution.root_seg_off.p.y(temp_vector(1));
			solution.root_seg_off.p.z(temp_vector(2));
		}

		if (handle.FirstChildElement("Root").FirstChildElement("quaternion").ToElement())
		{
			Eigen::VectorXd temp_vector;
			if (!xmlGetVector(*(handle.FirstChildElement("Root").FirstChildElement("quaternion").ToElement()), temp_vector))
			{
#ifdef KIN_DEBUG_MODE
				std::cout<<"Get root quaternion failed"<<std::endl;
#endif
				return false;
			}
			if (temp_vector.size() != 4)
			{
#ifdef KIN_DEBUG_MODE
				std::cout<<"Root quaternion vector size is invalid"<<std::endl;
#endif
				return false;
			}
			solution.root_seg_off.M =
					KDL::Rotation::Quaternion(temp_vector(1), temp_vector(2), temp_vector(3), temp_vector(0));
		}
	}

	solution.zero_other_joints = true;  //!< By default it is true
	if (handle.FirstChildElement("Update").ToElement()->Attribute("zero_unnamed")) //!< If it exists
	{
		if (handle.FirstChildElement("Update").ToElement()->QueryBoolAttribute("zero_unnamed", &solution.zero_other_joints)
				!= tinyxml2::XML_NO_ERROR)
		{
#ifdef KIN_DEBUG_MODE
			std::cout<<"Update joints are not properly defined"<<std::endl;
#endif
			return false;
		}  //!< If exists but wrongly defined
	}
	tinyxml2::XMLHandle joint_handle(handle.FirstChildElement("Update").FirstChildElement("joint"));
	while (joint_handle.ToElement())
	{
		if (!joint_handle.ToElement()->Attribute("name"))
		{
#ifdef KIN_DEBUG_MODE
			std::cout<<"Update joint names are invalid"<<std::endl;
#endif
			return false;
		} //!< If no name exists
		solution.joints_update.push_back(joint_handle.ToElement()->Attribute("name"));
		joint_handle = joint_handle.NextSiblingElement("joint");
	}
	if (solution.joints_update.size() < 1)
	{
#ifdef KIN_DEBUG_MODE
		std::cout<<"No update joint is specified"<<std::endl;
#endif
		return false;
	}  //!< If no joints specified

	solution.ignore_unused_segs = true;
	if (handle.FirstChildElement("EndEffector").ToElement())
	{
		if (handle.FirstChildElement("EndEffector").ToElement()->Attribute("ignore_unused"))
		{
			if (handle.FirstChildElement("EndEffector").ToElement()->QueryBoolAttribute("ignore_unused", &solution.ignore_unused_segs)
					!= tinyxml2::XML_NO_ERROR)
			{
#ifdef KIN_DEBUG_MODE
				std::cout<<"Invalid end-effector"<<std::endl;
#endif
				return false;
			}
		}
		tinyxml2::XMLHandle segment_handle(handle.FirstChildElement("EndEffector").FirstChildElement("limb"));
		while (segment_handle.ToElement())
		{
			if (!segment_handle.ToElement()->Attribute("segment"))
			{
#ifdef KIN_DEBUG_MODE
				std::cout<<"Invalid end-effector segment"<<std::endl;
#endif
				return false;
			}
			solution.end_effector_segs.push_back(segment_handle.ToElement()->Attribute("segment"));
			KDL::Frame temp_frame = KDL::Frame::Identity(); //!< Initialise to identity
			if (segment_handle.FirstChildElement("vector").ToElement())
			{
				Eigen::VectorXd temp_vector;
				if (!xmlGetVector(*(segment_handle.FirstChildElement("vector").ToElement()), temp_vector))
				{
#ifdef KIN_DEBUG_MODE
					std::cout<<"Invalid end-effector offset position vector"<<std::endl;
#endif
					return false;
				}
				if (temp_vector.size() != 3)
				{
#ifdef KIN_DEBUG_MODE
					std::cout<<"Invalid end-effector offset position vector size"<<std::endl;
#endif
					return false;
				}
				temp_frame.p.x(temp_vector(0));
				temp_frame.p.y(temp_vector(1));
				temp_frame.p.z(temp_vector(2));
			}
			if (segment_handle.FirstChildElement("quaternion").ToElement())
			{
				Eigen::VectorXd temp_vector;
				if (!xmlGetVector(*(segment_handle.FirstChildElement("quaternion").ToElement()), temp_vector))
				{
#ifdef KIN_DEBUG_MODE
					std::cout<<"Invalid end-effector offset quaternion vector"<<std::endl;
#endif
					return false;
				}
				if (temp_vector.size() != 4)
				{
#ifdef KIN_DEBUG_MODE
					std::cout<<"invalid end-effector offset quaternion vector size"<<std::endl;
#endif
					return false;
				}
				temp_frame.M =
						KDL::Rotation::Quaternion(temp_vector(1), temp_vector(2), temp_vector(3), temp_vector(0));
			}
			solution.end_effector_offs.push_back(temp_frame);
			segment_handle = segment_handle.NextSiblingElement("limb");
		}
	}
	bool success = initKinematics(urdf_file, solution);
#ifdef KIN_DEBUG_MODE
	if(success)
	std::cout<<"Kinematica Initialisation From XML SUCCEEDED"<<std::endl;
#endif
	return success;
}
bool kinematica::KinematicTree::updateEndEffectors(const SolutionForm_t & new_end_effectors)
{
//!< Lock for synchronisation
	boost::mutex::scoped_lock(member_lock_);

//!< Clear the needed flag and the end-effector segments vectors
	for (int i = 0; i < robot_tree_.size(); ++i)
	{
		robot_tree_[i].needed = false;
	}
	eff_segments_.clear();
	eff_seg_offs_.clear();

//!< Now do the actual updating
	return setEndEffectors(new_end_effectors);
}

bool kinematica::KinematicTree::updateEndEffectorOffsets(const std::vector<int> & index,
		const std::vector<KDL::Frame> & offset)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
		return false;
	if (index.size() > eff_seg_offs_.size() || index.size() != offset.size())
		return false;
	for (int i = 0; i < index.size(); i++)
		eff_seg_offs_[i] = offset[i];
	return true;
}

bool kinematica::KinematicTree::getEndEffectorIndex(std::vector<int> & eff_index)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
		return false;
	eff_index = eff_segments_;
	return true;
}

bool kinematica::KinematicTree::addEndEffector(const std::string & name, const KDL::Frame & offset)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}
//!< Check if the end-effector is a segment
	std::map<std::string, int>::iterator map_it = segment_map_.find(name);
	if (map_it == segment_map_.end())
	{
		return false;
	}

	uint N = eff_segments_.size(), i;
	for (i = 0; i < N; i++)
	{
		if (map_it->second == eff_segments_[i])
		{
			return false;
		}
	}
	if (!recurseNeedFlag(segment_map_[name]))
	{
		return false;
	}
	eff_segments_.push_back(segment_map_[name]);
	eff_seg_offs_.push_back(offset);
	forward_map_.resize(3 * eff_segments_.size());	//!< Just position/velocity of end-effector
	jacobian_.resize(3 * eff_segments_.size(), num_jnts_spec_);
	return true;
}

bool kinematica::KinematicTree::removeEndEffector(const std::string & name)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}
//!< Check if the end-effector is a segment
	std::map<std::string, int>::iterator map_it = segment_map_.find(name);
	if (map_it == segment_map_.end())
		return false;
	uint N = eff_segments_.size(), i;
	for (i = 0; i < N; i++)
	{
		if (eff_segments_[i] == map_it->second)
		{
			eff_segments_.erase(eff_segments_.begin() + i);
			eff_seg_offs_.erase(eff_seg_offs_.begin() + i);
			for (int i = 0; i < robot_tree_.size(); ++i)
			{
				robot_tree_[i].needed = false;
			}

			for (i = 0; i < N - 1; i++)
			{
				if (!recurseNeedFlag(eff_segments_[i]))
				{
					return false;
				}
			}
			forward_map_.resize(3 * (N - 1));	//!< Just position/velocity of end-effector
			jacobian_.resize(3 * (N - 1), num_jnts_spec_);
			return true;
		}

	}
	return false;
}

bool kinematica::KinematicTree::modifyEndEffector(const std::string & name,
		const KDL::Frame & offset)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}
//!< Check if the end-effector is a segment
	std::map<std::string, int>::iterator map_it = segment_map_.find(name);
	if (map_it == segment_map_.end())
		return false;
	uint index = map_it->second;
	uint N = eff_segments_.size(), i;
	for (i = 0; i < N; i++)
	{
		if (eff_segments_[i] == index)
		{
			eff_seg_offs_[i] = offset;
			return true;
		}
	}
	return false;
}

bool kinematica::KinematicTree::updateConfiguration(
		const Eigen::Ref<const Eigen::VectorXd> & joint_configuration)
{
//!< Temporaries
	double jnt_angle;

	//!< Locking
	boost::mutex::scoped_lock(member_lock_);

//!< Checks
	if (!isInitialised())
	{
		ERROR("Scene was not initialized!");
		return false;
	}
	if (!zero_undef_jnts_ && joint_configuration.size() != num_jnts_spec_)
	{
		ERROR("Joint vector size is incorrect!\nExpected "<<joint_configuration.size()<<", found "<<num_jnts_spec_);
		return false;
	}

//!< Start update: start first with the root node...
	jnt_angle =
			(robot_tree_[0].joint_type == JNT_UNUSED) ? 0 : joint_configuration[robot_tree_[0].joint_index];
	robot_tree_[0].joint_pose = robot_tree_[0].tip_pose
			* robot_tree_[0].segment.pose(jnt_angle).Inverse();
	if (robot_tree_[0].joint_type)	//!< Will be greater than 0
	{
		robot_tree_[0].joint_origin = vectorKdlToEigen(robot_tree_[0].joint_pose
				* robot_tree_[0].segment.getJoint().JointOrigin());
		robot_tree_[0].joint_axis = vectorKdlToEigen(robot_tree_[0].joint_pose.M
				* robot_tree_[0].segment.getJoint().JointAxis());
		if (!robot_tree_[0].to_tip)
		{
			robot_tree_[0].joint_axis = -1.0 * robot_tree_[0].joint_axis;
		}
	}

	for (int i = 1; i < robot_tree_.size(); i++)
	{
		if (robot_tree_[i].needed)	//!< Only proceed if needed
		{
			//!< Temporaries
			KDL::Frame parent_transform;

			//!< Settle Angle
			jnt_angle =
					(robot_tree_[i].joint_type == JNT_UNUSED) ? 0 : joint_configuration[robot_tree_[i].joint_index];

			//!< Settle which parent transform to use
			if (robot_tree_[i].from_tip)	//If we are coming from the tip of the parent
			{
				parent_transform = robot_tree_[robot_tree_[i].parent].tip_pose;
			}
			else
			{
				parent_transform = robot_tree_[robot_tree_[i].parent].joint_pose;
			}

			//!< Now settle the tip or base
			if (robot_tree_[i].to_tip)	//!< We are moving towards the tip
			{//!< We generally do not need to concern ourselves with the joint_pose: however might need it for the joint origin computation
				robot_tree_[i].tip_pose = parent_transform * robot_tree_[i].segment.pose(jnt_angle);
				robot_tree_[i].joint_pose = parent_transform;
			}
			else //!< Moving towards the base
			{
				robot_tree_[i].tip_pose = parent_transform;	//!< We cannot be moving towards base from a tip
				robot_tree_[i].joint_pose = parent_transform
						* robot_tree_[i].segment.pose(jnt_angle).Inverse();
			}

			//!< Finally set the joint_origin/axis
			if (robot_tree_[i].joint_type)//!< If we are specifying this joint: if not, it does not make sense to compute it
			{
				if (robot_tree_[i].joint_type == JNT_ROTARY)  //!< Only compute axis if rotary
				{
					robot_tree_[i].joint_origin = vectorKdlToEigen(robot_tree_[i].joint_pose
							* robot_tree_[i].segment.getJoint().JointOrigin()); //!< Origin is just transformed into the global frame
				}
				robot_tree_[i].joint_axis = vectorKdlToEigen(robot_tree_[i].joint_pose.M
						* robot_tree_[i].segment.getJoint().JointAxis()); //!< For the axes, we do not care about co-ordinates: we just pre-multiply by rotation: this is needed always, for both rotary and prismatic joints
				if (!robot_tree_[i].to_tip)
				{
					robot_tree_[i].joint_axis = -1.0 * robot_tree_[i].joint_axis;
				}
			}
		}
	}
	return true;
}

bool kinematica::KinematicTree::generateForwardMap()
{
	boost::mutex::scoped_lock(member_lock_);	//!< Lock:
	return computePhi();
}

bool kinematica::KinematicTree::generateForwardMap(Eigen::Ref<Eigen::VectorXd> phi)
{
	boost::mutex::scoped_lock(member_lock_);	//!< Lock for thread-safety

	if (computePhi())	//!< If successful:
	{
		if (phi.rows() != forward_map_.rows())
		{
			ERROR("Return vector has wrong size!");
			return false;
		}
		phi = forward_map_;
		return true;
	}
	else
	{
		return false;
	}
}

bool kinematica::KinematicTree::getPhi(Eigen::Ref<Eigen::VectorXd> phi)
{
	boost::mutex::scoped_lock(member_lock_);	//!< Lock for thread-safety
	if (phi.rows() != forward_map_.rows())
	{
		ERROR("Return vector has wrong size ("<<phi.rows()<<"!="<<forward_map_.rows()<<")!");
		return false;
	}
	phi = forward_map_;
	return true;
}

bool kinematica::KinematicTree::generateJacobian()
{
	boost::mutex::scoped_lock(member_lock_); 	//!< Locking
	return computePosJacobian();
}

bool kinematica::KinematicTree::getJacobian(Eigen::Ref<Eigen::MatrixXd> jac)
{
	boost::mutex::scoped_lock(member_lock_);	//!< Lock for thread-safety
	if (jac.rows() != jacobian_.rows() || jac.cols() != jacobian_.cols())
	{
		std::cout << "Has " << jacobian_.rows() << "X" << jacobian_.cols() << ". Req: "
				<< jac.rows() << "x" << jac.cols() << std::endl;
		ERROR("Return matrix has wrong size!");
		return false;
	}
	jac = jacobian_;
	return true;
}

bool kinematica::KinematicTree::generateJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian)
{
	boost::mutex::scoped_lock(member_lock_); 	//!< Locking

	if (computePosJacobian()) 	//!< If ok...
	{
		if (jacobian.rows() != jacobian_.rows() || jacobian.cols() != jacobian_.cols())
		{
			ERROR("Return matrix has wrong size! Required size "<<jacobian.rows()<<"x"<<jacobian.cols()<<". Has size"<<jacobian_.rows()<<"x"<<jacobian_.cols());

			return false;
		}
		jacobian = jacobian_;
		return true;
	}
	else
	{
		return false;
	}
}

bool kinematica::KinematicTree::generateCoM()
{

	if (!isInitialised())
	{
		return false;
	}
	uint N = robot_tree_.size(), i;
	double M = 0, m;
	KDL::Vector com = KDL::Vector::Zero();
	KDL::Frame tip_pose;
	for (i = 0; i < N; i++)
	{
		m = robot_tree_[i].segment.getInertia().getMass();
		M = M + m;
		tip_pose = robot_tree_[i].joint_pose
				* KDL::Frame(robot_tree_[i].segment.getInertia().getCOG());
		com = com + m * tip_pose.p;
	}

	com = com / M;
	com_.x() = com.x();
	com_.y() = com.y();
	com_.z() = com.z();
	return true;
}
bool kinematica::KinematicTree::getCoMProperties(std::vector<std::string> & segs,
		Eigen::VectorXd & mass, std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
		std::vector<KDL::Frame> & base_pose)
{
	if (!isInitialised())
	{
		return false;
	}
	if (!generateCoM())
	{
		return false;
	}
	uint NTotal = robot_tree_.size(), i;
	mass.resize(num_jnts_spec_);
	cog.resize(num_jnts_spec_);
	tip_pose.resize(num_jnts_spec_);
	base_pose.resize(num_jnts_spec_);
	segs.resize(num_jnts_spec_);
	for (i = 0; i < eff_segments_.size(); i++)
	{
		segs[i] = robot_tree_[eff_segments_[i]].segment.getName();
		mass(i) = robot_tree_[eff_segments_[i]].segment.getInertia().getMass();
		cog[i] = robot_tree_[eff_segments_[i]].segment.getInertia().getCOG();
		tip_pose[i] = robot_tree_[eff_segments_[i]].tip_pose;
		base_pose[i] = robot_tree_[eff_segments_[i]].joint_pose;
	}
	return true;
}
bool kinematica::KinematicTree::getSegment(KDL::Segment & seg, int index)
{
	boost::mutex::scoped_lock(member_lock_);
	if (index < 0 || index >= robot_tree_.size() || !isInitialised())
	{
		return false;	//!< Not initialised, or invalid index
	}
	seg = robot_tree_[index].segment;
	return true;
}

bool kinematica::KinematicTree::getSegments(std::vector<KDL::Segment> & segs)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}
	uint N = robot_tree_.size(), i;
	segs.resize(N);
	for (i = 0; i < N; i++)
	{
		segs[i] = robot_tree_[i].segment;
	}
	return true;
}

bool kinematica::KinematicTree::getControlledSegmentsAndJoints(std::vector<std::string> & segs,
		std::vector<std::string> & joints)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}
	segs.resize(used_joints_segs_.size());
	segs = used_joints_segs_;
	joints.resize(num_jnts_spec_);
	joints = used_joints_;
	return true;
}

bool kinematica::KinematicTree::getInitialEff(std::vector<std::string> & segs,
		std::vector<KDL::Frame> & offsets)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}
	segs.resize(eff_segments_ini_.size());
	offsets.resize(eff_segments_ini_.size());
	segs = eff_segments_ini_;
	offsets = eff_seg_offs_ini_;
	return true;
}
bool kinematica::KinematicTree::initialise(const KDL::Tree & temp_tree,
		const kinematica::SolutionForm_t & optimisation)
{
	std::map<std::string, int> joint_map; //!< Mapping from joint names to index of associated segment
	bool success; //!< Running measure of success

//!< First clear/reset everything
	robot_tree_.clear();
	segment_map_.clear();
	zero_undef_jnts_ = false;
	num_jnts_spec_ = 0;
	eff_segments_.clear();
	eff_seg_offs_.clear();

//!< Attempt to Build the Tree
	success = buildTree(temp_tree, optimisation.root_segment, joint_map);

#ifdef KIN_DEBUG_MODE
	if (success) std::cout << "Initialisation Function ... Built Internal Tree" << std::endl;
#endif

//!< Set the Joint ordering
	if (success)
	{
		success =
				setJointOrder(optimisation.joints_update, optimisation.zero_other_joints, joint_map);
	}

#ifdef KIN_DEBUG_MODE
	if (success) std::cout << "Initialisation Function ... Defined Joint Ordering" << std::endl;
#endif

//!< Set the End-Effector Kinematics
	if (success)
	{
		success = setEndEffectors(optimisation);

	}

	eff_segments_ini_ = optimisation.end_effector_segs;
	eff_seg_offs_ini_ = optimisation.end_effector_offs;
#ifdef KIN_DEBUG_MODE
	if (success) std::cout << "Initialisation Function ... Set End Effectors" << std::endl;
#endif

//!< Clean up if necessary
	if (!success)
	{ //!< Clear up everything
		robot_tree_.clear();
		segment_map_.clear();
		zero_undef_jnts_ = false;
		num_jnts_spec_ = 0;
		eff_segments_.clear();
		eff_seg_offs_.clear();
	}

#ifdef KIN_DEBUG_MODE
	if (success) std::cout << "Initialisation Function ... returning" << std::endl;
#endif
	return success;
}

bool kinematica::KinematicTree::buildTree(const KDL::Tree & temp_tree, std::string root,
		std::map<std::string, int> & joint_map)
{
#ifdef KIN_DEBUG_MODE
	std::cout << "buildTree Function ... " << std::endl;
#endif

//!< Variable Declarations
	KDL::SegmentMap::const_iterator root_segment; //!< Root segment iterator
	std::string true_root; //!< The urdf root name
	int rubbish; //!< Garbage value since we know root will be at 0

//!< Get the desired segment as root...
	if (root.size() == 0) //!< If no root specified, then we will use the urdf root
	{
#ifdef KIN_DEBUG_MODE
		std::cout << "buildTree Function ... root is of size 0" << std::endl;
#endif
		root_segment = temp_tree.getRootSegment();
#ifdef KIN_DEBUG_MODE
		std::cout << "buildTree Function ... root name: " << root_segment->second.segment.getName() << std::endl;
#endif
	}
	else                                 //!< We first need to check if this root actually exists...
	{
#ifdef KIN_DEBUG_MODE
		std::cout << "buildTree Function ... root is " << root << std::endl;
#endif
		KDL::SegmentMap tree_segments = temp_tree.getSegments(); //!< Map of tree segments for checking that desired segment actually exists
		if (tree_segments.find(root) != tree_segments.end()) //!< If it exists...
		{
			root_segment = temp_tree.getSegment(root);
		}
		else
		{
			return false; //!< Indicate failure
		}
	}
	true_root = temp_tree.getRootSegment()->second.segment.getName();
	return addSegment(root_segment, ROOT, rubbish, true, false, true_root, joint_map); //!< We do a little trick here to indicate that this is the root node
}

bool kinematica::KinematicTree::setJointOrder(const std::vector<std::string> & joints,
		bool zero_out, const std::map<std::string, int> & joint_map)
{
//!< First do some checks
	if (!zero_out && joints.size() != joint_map.size())
	{
		return false;
	}

//!< Now fill in the joints that will be specified: rest will still be invalidated
	num_jnts_spec_ = joints.size();
	used_joints_.resize(num_jnts_spec_);
	used_joints_segs_.resize(num_jnts_spec_);
	used_joints_ = joints;
	for (int i = 0; i < num_jnts_spec_; i++)
	{
		if (joint_map.find(joints[i]) == joint_map.end()) //!< Guard against possibility of specifying incorrect joints
		{
#ifdef KIN_DEBUG_MODE
			std::cout << "buildTree Function ...  could not find joint " << joints[i] << std::endl;
#endif
			return false;
		}
		//!< First check what type of joint it is:
		switch (robot_tree_[joint_map.at(joints[i])].segment.getJoint().getType())
		{
			case KDL::Joint::RotAxis:
			case KDL::Joint::RotX:
			case KDL::Joint::RotY:
			case KDL::Joint::RotZ:
				robot_tree_[joint_map.at(joints[i])].joint_type = JNT_ROTARY;
				break;

			case KDL::Joint::TransAxis:
			case KDL::Joint::TransX:
			case KDL::Joint::TransY:
			case KDL::Joint::TransZ:
				robot_tree_[joint_map.at(joints[i])].joint_type = JNT_PRISMATIC;
				break;

			default:
				return false;
				break;
		}
		//!< Now set the joint index
		robot_tree_[joint_map.at(joints[i])].joint_index = i;
		used_joints_segs_[i] = robot_tree_[joint_map.at(joints[i])].segment.getName();
	}

	return true;
}

bool kinematica::KinematicTree::setEndEffectors(const SolutionForm_t & optimisation)
{
//!< Variable Declaration
	bool success = true;
#ifdef KIN_DEBUG_MODE
	std::cout << "setEndEffectors Function ...  Entered with offsets of size " << optimisation.end_effector_offs.size() << std::endl;
#endif
//!< First do some checks
	if (optimisation.end_effector_offs.size() < 0 //OK if == 0
	&& (optimisation.end_effector_segs.size() != optimisation.end_effector_offs.size()))
	{
		return false;
	}

#ifdef KIN_DEBUG_MODE
	std::cout << "setEndEffectors Function ...  Sizes match up OK!" << std::endl;
#endif

//!< Initialise
	robot_tree_[0].tip_pose = (optimisation.root_seg_off).Inverse(); //!< The Root's Tip Pose is the inverse of the given transformation: this will be constant!
	used_joints_segs_ = optimisation.end_effector_segs;

	for (int i = 0; i < optimisation.end_effector_segs.size() && success; i++)
	{
		if (segment_map_.find(optimisation.end_effector_segs[i]) != segment_map_.end())
		{
			eff_segments_.push_back(segment_map_[optimisation.end_effector_segs[i]]); //!< Push back the index
			if (optimisation.end_effector_offs.size())
			{
				eff_seg_offs_.push_back(optimisation.end_effector_offs[i]);
			} //!< If larger than 0, push back the frame offset
			success =
					recurseNeedFlag(segment_map_[optimisation.end_effector_segs[i]]); //!< Set the needed flag for this and all parents
#ifdef KIN_DEBUG_MODE
							std::cout << "setEndEffectors Function ...  Managed to add End effector " << optimisation.end_effector_segs[i] << std::endl;
#endif
		}
		else
		{
#ifdef KIN_DEBUG_MODE
			std::cout << "setEndEffectors Function ...  Could not use End effector " << optimisation.end_effector_segs[i] << std::endl;
#endif
			success = false;
		}
	}

	if (success)
	{
		forward_map_.resize(3 * optimisation.end_effector_segs.size());	//!< Just position/velocity of end-effector
		jacobian_.resize(3 * optimisation.end_effector_segs.size(), num_jnts_spec_);

#ifdef KIN_DEBUG_MODE
		std::cout << "setEndEffectors Function ... Created Jacobian of size " << 3*optimisation.end_effector_segs.size() << " x " << num_jnts_spec_ << std::endl;
#endif
		if (!optimisation.ignore_unused_segs) //!< If we do not wish to ignore unused chains
		{
			for (int i = 0; i < robot_tree_.size(); i++)
			{
				robot_tree_[i].needed = true;
			} //!< Set all as needed
		}
	}

	return success;
}

std::string kinematica::KinematicTree::getRootName()
{
	return robot_tree_[0].segment.getName();
}

bool kinematica::KinematicTree::modifyRootOffset(KDL::Frame & offset)
{
	if (!isInitialised())
	{
		ERROR("Kinematic tree was not initialized!");
		return false;
	}
	robot_tree_[0].tip_pose = offset.Inverse();
	return true;
}
bool kinematica::KinematicTree::addSegment(KDL::SegmentMap::const_iterator current_segment,
		int parent, int & current, bool from_ptip, bool to_ctip, const std::string & root_name,
		std::map<std::string, int> & joint_map)
{
//!< Variable Declaration
	bool success = true;
	KinematicElement_t current_node;

#ifdef KIN_DEBUG_MODE
	std::cout << "addSegment Function ... with " << current_segment->second.segment.getName() << " parent: " << parent << " flags: " << from_ptip << to_ctip << " root: " << root_name << std::endl;
#endif

//!< First fill in this node
	current_node.parent = parent; //!< Assign the parent
	current_node.from_tip = from_ptip; //!< Indicate whether we reached this through the tip (true) or the base of the parent
	current_node.to_tip = to_ctip; //!< Also check whether we are moving towards the tip or the base
	current_node.segment = current_segment->second.segment; //!< Assign the segment information
	current_node.joint_type = JNT_UNUSED;
	current_node.needed = false; //!< By default not needed
	current = robot_tree_.size(); //!< Set where this node will be stored
	robot_tree_.push_back(current_node); //!< Store

#ifdef KIN_DEBUG_MODE
	std::cout << "addSegment Function ... created node and pushed back on tree at " << robot_tree_.size() - 1 << std::endl;
#endif

//!< Update the Segment Map and the Joint Map:
	segment_map_[current_node.segment.getName()] = current;
	joint_map[current_node.segment.getJoint().getName()] = current;

#ifdef KIN_DEBUG_MODE
	std::cout << "addSegment Function ... Indexing Segment and joint maps (" << current_node.segment.getJoint().getName() << ")" << std::endl;
#endif

//!< Now comes the tricky part:
	if (to_ctip) //!< We are moving in the forward direction towards the tip (this was a child of the node calling the function)
	{
#ifdef KIN_DEBUG_MODE
		std::cout << "addSegment Function ... Moving to a tip " << std::endl;
#endif
		//!< First Iterate through children
		for (int i = 0; i < current_segment->second.children.size() && success; i++) //!< Iterate through the children if any
		{
#ifdef KIN_DEBUG_MODE
			std::cout << "addSegment Function ... Iterating through children: " << i << std::endl;
#endif
			int child;
			success =
					addSegment(current_segment->second.children[i], current, child, true, true, root_name, joint_map); //!< We are moving from tip towards a tip
			robot_tree_[current].child.push_back(child); //!< Assign Child to this node
		}
		//!< Base Case: If empty, loop will be skipped
	}
	else //!< We are moving towards the base
	{
#ifdef KIN_DEBUG_MODE
		std::cout << "addSegment Function ... Moving to a base " << std::endl;
#endif
		if (from_ptip) //!< This combination (from tip but moving to a base) is impossible, but is used to indicate this is the root node
		{
#ifdef KIN_DEBUG_MODE
			std::cout << "addSegment Function ... Manipulating Root segment " << std::endl;
#endif
			//!< Iterate first through children
			for (int i = 0; i < current_segment->second.children.size() && success; i++) //!< Iterate through the children if any
			{
#ifdef KIN_DEBUG_MODE
				std::cout << "addSegment Function ... Iterating through children of root: " << i << std::endl;
#endif
				int child;
				success =
						addSegment(current_segment->second.children[i], current, child, true, true, root_name, joint_map); //!< We are moving from tip towards a tip
				robot_tree_[current].child.push_back(child); //!< Assign Child to this node
			}
			//!< Now handle the parent: only if previously successfull and if this is not the original root node
			if (root_name.compare(current_node.segment.getName()) && success)
			{
#ifdef KIN_DEBUG_MODE
				std::cout << "addSegment Function ... Checking parent of root " << std::endl;
#endif
				int child;
				success =
						addSegment(current_segment->second.parent, current, child, false, false, root_name, joint_map);	//!< We are moving from base towards base
				robot_tree_[current].child.push_back(child);	//!< Add as child
			}
		}
		else	//!< I.e. we Are moving from base to a base
		{
#ifdef KIN_DEBUG_MODE
			std::cout << "addSegment Function ... Moving from base to base: "<< std::endl;
#endif
			//!< Iterate through children and set them as children of parent rather than current
			for (int i = 0; i < current_segment->second.children.size() && success; i++)
			{
#ifdef KIN_DEBUG_MODE
				std::cout << "addSegment Function ... Iterating through children of an inverted segment: " << i << std::endl;
#endif
				int child;
				std::string child_name =
						current_segment->second.children[i]->second.segment.getName();//!< The name of this child
				std::string parent_name = robot_tree_[parent].segment.getName();//!< The name of the parent
				if (parent_name.compare(child_name) == 0)
				{
					continue;
				}									//!< Skip the child who is now parent
				success =
						addSegment(current_segment->second.children[i], parent, child, false, true, root_name, joint_map);
				robot_tree_[parent].child.push_back(child);			//!< Assign child to parent node
			}
			//!< If empty, loop will be skipped
			if (root_name.compare(current_node.segment.getName()) && success)//!< IF not equal to the root
			{
#ifdef KIN_DEBUG_MODE
				std::cout << "addSegment Function ... Handling parent of inverted segment: " << std::endl;
#endif
				int child;
				success =
						addSegment(current_segment->second.parent, current, child, false, false, root_name, joint_map); //!< Add its parent as its child, but indicate so in the traversal direction
				robot_tree_[current].child.push_back(child);
			}
			//!< Base case if this is indeed the root node in the original urdf
		}
	}
	return success;
}

bool kinematica::KinematicTree::recurseNeedFlag(int node)
{
	robot_tree_[node].needed = true;	//!< Indicate that needed
//!< Handle Base Case first:
	if (robot_tree_[node].parent == ROOT)
	{
		return true;
	}
//!< Else, recurse
	else
	{
		return recurseNeedFlag(robot_tree_[node].parent);
	}
}

int kinematica::KinematicTree::getEffSize()
{
	return eff_segments_.size();
}

bool kinematica::KinematicTree::computePhi()
{
//!< Checks
	if (!isInitialised())
	{
		ERROR("Kinematic tree was not initialized!");
		return false;
	}

	for (int i = 0; i < eff_segments_.size(); i++)
	{
		KDL::Frame end_effector = robot_tree_[eff_segments_[i]].tip_pose;//!< End effector is w.r.t. tip of segment always
		if (eff_seg_offs_.size())	//!< if Size is greater than 0
		{
			end_effector = end_effector * eff_seg_offs_[i];	//!< Append the respective final transformation
		}
		forward_map_(i * 3) = end_effector.p.x();
		forward_map_(i * 3 + 1) = end_effector.p.y();
		forward_map_(i * 3 + 2) = end_effector.p.z();
	}
	return true;
}

bool kinematica::KinematicTree::computePosJacobian()
{
//!< Checks
	if (!isInitialised())
	{
		INDICATE_FAILURE
		;
		return false;
	}		//!< Ensure that Variables are initialised
	if (!forward_map_.size())
	{
		INDICATE_FAILURE
		;
		return false;
	}	//!< Ensure that forward_map_ is generated

//!< Compute Jacobian for each end-effector:
	jacobian_.fill(0.0);	//!< Reset everything to 0
	for (int i = 0; i < eff_segments_.size(); i++)	//!< Iterate through the end-effectors
	{
		//!< Temporaries
		Eigen::Vector3d end_effector_pos = forward_map_.segment(i * 3, 3);//!< Global End-effector position
		int segment_index = eff_segments_[i];	//!< Current segment index
		while (robot_tree_[segment_index].parent != ROOT)//!< Repeat until you reach the root joint
		{
			//!< Some tricks: basically, in some cases we may need to execute for both...
			if (robot_tree_[segment_index].to_tip)//!< If in traversing through node we go from base to tip, then we need to consider the effect of its joint
			{
				if (robot_tree_[segment_index].joint_type == JNT_ROTARY)
				{
					Eigen::Vector3d diff_vector = end_effector_pos
							- robot_tree_[segment_index].joint_origin;	//!< Find the p-vector
					jacobian_.block(i * 3, robot_tree_[segment_index].joint_index, 3, 1) =
							robot_tree_[segment_index].joint_axis.cross(diff_vector);//!< The Jacobian for this joint
				}
				else if (robot_tree_[segment_index].joint_type == JNT_PRISMATIC)
				{
					jacobian_.block(i * 3, robot_tree_[segment_index].joint_index, 3, 1) =
							robot_tree_[segment_index].joint_axis; //!< Just the axis
				}
				//!< Else will not be considered
			}
			if (!robot_tree_[segment_index].from_tip) //!< If we are connected to parent from its base, then we (also) need to consider the parents' joint, which also moves us...
			{
				if (robot_tree_[robot_tree_[segment_index].parent].joint_type == JNT_ROTARY)
				{
					Eigen::Vector3d diff_vector = end_effector_pos
							- robot_tree_[robot_tree_[segment_index].parent].joint_origin; //!< Find the p-vector (now with its parent joint)
					jacobian_.block(i * 3, robot_tree_[robot_tree_[segment_index].parent].joint_index, 3, 1) =
							robot_tree_[robot_tree_[segment_index].parent].joint_axis.cross(diff_vector); //!< The Jacobian for this joint
				}
				else if (robot_tree_[robot_tree_[segment_index].parent].joint_type == JNT_PRISMATIC)
				{
					jacobian_.block(i * 3, robot_tree_[robot_tree_[segment_index].parent].joint_index, 3, 1) =
							robot_tree_[robot_tree_[segment_index].parent].joint_axis;
				}
			}
			segment_index = robot_tree_[segment_index].parent;			//!< Move to its parent
		}
	}

//!< If made it this far:
	return true;
}

bool kinematica::KinematicTree::getPose(std::string child, std::string parent, KDL::Frame & pose)
{
//!< Synchronisation
	boost::mutex::scoped_lock(member_lock_);

//!< Checks
	if (!isInitialised())
	{
		return false;
	}
	if (segment_map_.find(child) == segment_map_.end())
	{
		return false;
	}
	if (segment_map_.find(parent) == segment_map_.end())
	{
		return false;
	}

//!< Computation
	pose = robot_tree_[segment_map_[parent]].tip_pose.Inverse()
			* robot_tree_[segment_map_[child]].tip_pose;

//!< Return
	return true;
}

bool kinematica::KinematicTree::getPose(std::string child, KDL::Frame & pose)
{
//!< Synchronisation
	boost::mutex::scoped_lock(member_lock_);

//!< Checks
	if (!isInitialised())
	{
		return false;
	}
	if (segment_map_.find(child) == segment_map_.end())
	{
		return false;
	}

//!< Computation
	pose = robot_tree_[segment_map_[child]].tip_pose;

//!< Return
	return true;
}

bool kinematica::KinematicTree::getPose(int child, int parent, KDL::Frame & pose)
{
	boost::mutex::scoped_lock(member_lock_);

	if (!isInitialised())
	{
		return false;
	}
	if (child < 0 || child > robot_tree_.size())
	{
		return false;
	}
	if (parent < 0 || parent > robot_tree_.size())
	{
		return false;
	}

	pose = robot_tree_[parent].tip_pose.Inverse() * robot_tree_[child].tip_pose;

	return true;
}

bool kinematica::KinematicTree::getPose(int child, KDL::Frame & pose)
{
	boost::mutex::scoped_lock(member_lock_);

	if (!isInitialised())
	{
		return false;
	}
	if (child < 0 || child > robot_tree_.size())
	{
		return false;
	}

	pose = robot_tree_[child].tip_pose;

	return true;
}

bool kinematica::KinematicTree::getSegmentMap(std::map<std::string, int> & segment_map)
{
	boost::mutex::scoped_lock(member_lock_);
	if (!isInitialised())
	{
		return false;
	}

	segment_map = segment_map_;
	return true;

}

std::string kinematica::KinematicTree::getParent(std::string child)
{
	boost::mutex::scoped_lock(member_lock_);

	if (segment_map_.find(child) != segment_map_.end())
	{
		return robot_tree_[robot_tree_[segment_map_[child]].parent].segment.getName();
	}
	else
	{
		return "";  //!< Empty string
	}
}

int kinematica::KinematicTree::getParent(int child)
{
	boost::mutex::scoped_lock(member_lock_);

	if (child < robot_tree_.size() && child >= 0)
	{
		return robot_tree_[child].parent;
	}
	else
	{
		return -2;
	}
}

std::vector<std::string> kinematica::KinematicTree::getChildren(std::string parent)
{
	boost::mutex::scoped_lock(member_lock_);

	std::vector<std::string> children;
	if (segment_map_.find(parent) != segment_map_.end())
	{
		int num_chldrn = robot_tree_[segment_map_[parent]].child.size();
		for (int i = 0; i < num_chldrn; i++)
		{
			children.push_back(robot_tree_[robot_tree_[segment_map_[parent]].child[i]].segment.getName());
		}
	}
	return children; //!< Which may be an empty array
}

std::vector<int> kinematica::KinematicTree::getChildren(int parent)
{
	boost::mutex::scoped_lock(member_lock_);

	if (parent < robot_tree_.size() && parent >= 0)
	{
		return robot_tree_[parent].child;
	}
	else
	{
		std::vector<int> children;
		return children;
	}
}

Eigen::Vector3d kinematica::vectorKdlToEigen(const KDL::Vector & kdl_vec)
{
	Eigen::Vector3d eigen_vec;

	eigen_vec(0) = kdl_vec.x();
	eigen_vec(1) = kdl_vec.y();
	eigen_vec(2) = kdl_vec.z();

	return eigen_vec;
}

bool kinematica::recursivePrint(kinematica::KinematicTree & robot, std::string node,
		std::string tab)
{
	if (tab.size() == 0)
	{
		std::cout << node << std::endl;
	}
	else if (tab.back() == '|')
	{
		std::cout << tab << "-->" << node << std::endl;
	}
	else
	{
		std::cout << tab << "|-->" << node << std::endl;
	}

	std::vector<std::string> children = robot.getChildren(node);

	if (children.size() == 0)
	{
		return true;
	}

	tab.append("  ");
	if (children.size() > 1)
	{
		tab.append("|");
	}
	else
	{
		tab.append("  ");
	}

	bool success = true;
	for (int i = 0; i < children.size() && success; i++)
	{
		if (i == children.size() - 1) //!< if last element...
		{
			tab.resize(tab.size() - 1);
		}
		success = recursivePrint(robot, children[i], tab);
	}
	return success;
}

bool kinematica::xmlGetVector(const tinyxml2::XMLElement & xml_vector,
		Eigen::VectorXd & eigen_vector)
{
	//!< Temporaries
	double temp_entry;
	int i = 0;

	if (!xml_vector.GetText())
	{
		eigen_vector = Eigen::VectorXd(); //!< Null matrix again
		return false;
	}
	std::istringstream text_parser(xml_vector.GetText());

	//!< Initialise looping
	text_parser >> temp_entry;
	while (!(text_parser.fail() || text_parser.bad()))  //!< No commenting!
	{
		eigen_vector.conservativeResize(++i); //!< Allocate storage for this entry (by increasing i)
		eigen_vector(i - 1) = temp_entry;
		text_parser >> temp_entry;
	}
	return (i > 0) ? true : false;
}
