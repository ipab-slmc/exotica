#ifndef KINEMATIC_TREE_H
#define KINEMATIC_TREE_H

#include "tinyxml2/tinyxml2.h"
#include <boost/thread/mutex.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <Eigen/Eigen>
#include <map>
#include <string>
#include <vector>
#include <set>

#define ROOT  -1     //!< The value of the parent for the root segment

namespace kinematica
{
	/**
	 * \brief Defines the two types of supported joints
	 */
	enum JointType_t
	{
		JNT_UNUSED = 0,  //!< Undefined (will not be used)
		JNT_PRISMATIC = 1,  //!< Prismatic joint
		JNT_ROTARY = 2   //!< Rotary Joint
	};

	/**
	 * \brief DEPRECATED : Defines different types of Jacobian Computations but will not be used
	 */
	enum KinematicsType_t
	{
		KIN_ERROR = 0,  //!< No Jacobian Computation
		KIN_POSIT = 1, KIN_ROTAT = 2, KIN_FULL = 3
	};

	struct SolutionForm_t
	{
			std::string root_segment;       //!< The root segment
			KDL::Frame root_seg_off;		    //!< Offset from tip of root segment
			std::vector<std::string> joints_update;    //!< The vector of joints we will be updating
			bool zero_other_joints;  //!< Zero out the other joints not referenced
			bool ignore_unused_segs; //!< Flag to ignore unnecessary sub-chains from the tree
			bool compute_com;	//!< Flag to compute centre of mass
			std::string base_type;	//!< Flag to indicate if the base is floating or fixed
			std::vector<std::string> end_effector_segs; //!< The segments to which the end-effectors are attached
			std::vector<KDL::Frame> end_effector_offs; //!< End Effector Offsets from the segment of choice: must be empty or same size as the end_effector_segs
	};

	struct KinematicElement_t
	{
			KDL::Segment segment;  		//!< The actual segment
			bool needed;	//!< Flag to check whether actually needed or not for the FK Computation
			int parent;   		//!< The parent link (index)
			bool from_tip; //!< Whether it is connected to the tip of its parent (true) or to its base
			bool to_tip;		//!< Indicates whether we are moving towards the tip (true) or not
			JointType_t joint_type;   //!< Type of joint
			int joint_index; //!< The Joint connecting this limb to its ORIGINAL parent (mapping into the configuration array), if used
			KDL::Frame tip_pose;     //!< The latest update of the pose w.r.t root
			KDL::Frame joint_pose; //!< The pose of the base of the joint at the base of this segment (N.B. THIS IS THE POSE OF THE BASE OF THE FULL SEGMENT, from base of joint)
			Eigen::Vector3d joint_origin; //!< The Joint origin (actual position of joint-axis, in global frame)
			Eigen::Vector3d joint_axis;		//!< The Joint Axis of rotation (in the global frame)...
			std::vector<int> child;    		//!< Any child links
			Eigen::Vector3d com;	//!< Centre of mass
			std::vector<double> joint_limits_;
	};

	class KinematicTree
	{
		public:

			/**
			 * \brief Constructor: creates a kinematic tree from a urdf file (uses kdl parser internally)
			 * @param urdf_param     Name of the urdf ROS parameter
			 * @param optimisation  Optimisation Parameters
			 */
			KinematicTree(void);                      //!< Default Constructor
			KinematicTree(const std::string & urdf_param, const SolutionForm_t & optimisation); //!< Initialisation Constructor
			KinematicTree(const KDL::Tree & temp_tree, const SolutionForm_t & optimisation); //!< Initialisation Constructor
			KinematicTree(const KinematicTree & rhs);
			KinematicTree & operator=(const KinematicTree & rhs);

			~KinematicTree()
			{
			}
			;
			/**
			 * \brief Initialise the Kinematics Object. THREAD-SAFE
			 * @param urdf_param     Name of the urdf ROS parameter
			 * @param temp_tree     The KDL::Tree from which to construct the robot
			 * @param optimisation  Optimisation Parameters
			 * @return              True if successful, false otherwise
			 */
			bool initKinematics(const std::string & urdf_param,
					const SolutionForm_t & optimisation); //!< Variant taking urdf file
			bool initKinematics(const KDL::Tree & temp_tree, const SolutionForm_t & optimisation); //!< Variant taking kdl tree
			bool initKinematics(tinyxml2::XMLHandle & handle);
			bool initKinematics(tinyxml2::XMLHandle & handle,
					const robot_model::RobotModelPtr & model);

			/**
			 * \brief Provides dynamic updating of the end-effector list
			 * @param new_end_effectors A solution form type: you only need to set the ignore_unused_segs flag, the end_effector_segs and the end_effector_offs
			 * @return                  True if successful, false otherwise
			 */
			bool updateEndEffectors(const SolutionForm_t & new_end_effectors);

			/**
			 * \brief	Update end-effector offset
			 * @param	index	End-effector index
			 * @param	offset	New offset
			 */
			bool updateEndEffectorOffsets(const std::vector<int> & index,
					const std::vector<KDL::Frame> & offset);

			/**
			 * \brief	Add new end-effector
			 * @param	name	End-effector segment name
			 * @param	offset	End-effector offset
			 * @return	True if succeeded, false otherwise
			 */
			bool addEndEffector(const std::string & name, const KDL::Frame & offset);

			/**
			 * \brief	Remove end-effector
			 * @param	name	End-effector segment name
			 * @return	True if succeeded, false otherwise
			 */
			bool removeEndEffector(const std::string & name);

			/**
			 * \brief	Modify end-effector
			 * @param	name	End-effector segment name
			 * @param	offset	End-effector offset
			 * @return	True if succeeded, false otherwise
			 */
			bool modifyEndEffector(const std::string & name, const KDL::Frame & offset);

			/**
			 * \brief Update the internal position of joints pose of each segment : THREAD-SAFE
			 * @param joint_configuration The value for each joint in the order specified by joint ordering.
			 * @return                    True if successful, False if not
			 */
			bool updateConfiguration(const Eigen::Ref<const Eigen::VectorXd> & joint_configuration);

			/**
			 * \brief Generate the phi vector (forward kinematics for the specified end-effectors) : THREAD-SAFE
			 * @param phi	The Forward Kinematics Vector
			 * @return	True if successful, false otherwise
			 */
			bool generateForwardMap();
			bool generateForwardMap(Eigen::Ref<Eigen::VectorXd> phi);

			/**
			 * \brief Generate the Jacobian: assumes that the forward map is updated
			 * @param jacobian The Jacobian Matrix
			 * @return 		 True if successful, false otherwise
			 */
			bool generateJacobian();
			bool generateJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian);

			/**
			 * \brief	Generate the centre of mass position
			 * @param	com	Centre of mass
			 * @return	True if succeeded, false otherwise
			 */
			bool generateCoM();
			bool generateCoM(Eigen::Vector3d & com);
			/**
			 * \brief Returns the Jacobian that was computed : THREAD-SAFE
			 * @param jacobian  Reference to the Jacobian to modify
			 * @return          True if successful, false otherwise
			 */
			bool getJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian);

			/**
			 * \brief Returns the Forward Kinematics Map that was computed : THREAD-SAFE
			 * @param phi Reference to the FK Map to modify
			 * @return    True if successful, false otherwise
			 */
			bool getPhi(Eigen::Ref<Eigen::VectorXd> phi);

			/**
			 * \brief Computes and returns the transformation between a child and a parent link: THREAD-SAFE
			 * @param child   Name of the child link/Index of the child link
			 * @param parent  Name of the parent link/Index of the parent link: root frame (not root segment) if lacking
			 * @param pose    The frame transformation
			 * @return        True if no error, false otherwise
			 */
			bool getPose(std::string child, std::string parent, KDL::Frame & pose);
			bool getPose(std::string child, KDL::Frame & pose);
			bool getPose(int child, int parent, KDL::Frame & pose);
			bool getPose(int child, KDL::Frame & pose);

			/**
			 * \brief Returns the mapping between segment names and their indexes
			 * @param segment_map Reference to the map for storing the segment mapping
			 * @return            True if success, false otherwise
			 */
			bool getSegmentMap(std::map<std::string, int> & segment_map);

			/**
			 * \brief	Get the segments
			 * @param	seg		KDL Segment
			 * @param	segs	KDL Segments for all links
			 * @param	index	Index in the tree
			 * @return	True if succeeded, false otherwise
			 */
			bool getSegment(KDL::Segment & seg, int index);
			bool getSegments(std::vector<KDL::Segment> & segs);

			/**
			 * \brief	Get the controlled segments and joints names
			 * @param	segs	Segment names
			 * @param	joints	Joint names
			 * @return	True if succeeded, false otherwise
			 */
			bool getControlledSegmentsAndJoints(std::vector<std::string> & segs,
					std::vector<std::string> & joints);
			/**
			 * \brief	Get centre of mass properties
			 * @param	seg		Segment names
			 * @param	mass	Mass of each link
			 * @param	cog		Centre of gravity of each link
			 * @param	tip_pose	Tip pose of each link
			 * @param	base_pose	Base pose of each link
			 */
			bool getCoMProperties(const std::vector<int>& ids, std::vector<std::string> & segs,
					Eigen::VectorXd & mass, std::vector<KDL::Vector> & cog,
					std::vector<KDL::Frame> & tip_pose, std::vector<KDL::Frame> & base_pose);

			/**
			 * \brief Allows traversing the tree structure by getting the parent
			 * @param child   The Name of the child/Index into the array
			 * @return        The Parent (either string or index depending on call)
			 */
			std::string getParent(std::string child);
			int getParent(int child);

			/**
			 * \brief Allows traversing the tree structure by getting the children
			 * @param parent  The Name of the parent/Index into the array
			 * @return        The vector of children (either string or int index)
			 */
			std::vector<std::string> getChildren(std::string parent);
			std::vector<int> getChildren(int parent);

			/**
			 * \brief Returns the number of end-effectors.
			 * @return Number of end-effectors.
			 */
			int getEffSize();

			int getNumJoints();
			std::vector<std::string> & getJointNames()
			{
				return used_joints_;
			}
			;
			bool getInitialEff(std::vector<std::string> & segs, std::vector<KDL::Frame> & offsets);
			bool modifyRootOffset(KDL::Frame & offset);
			std::string getRootName();

			bool getEndEffectorIndex(std::vector<int> & eff_index);

			std::string getBaseType();
			std::map<std::string, int> getJointMap();
			std::map<std::string, std::vector<double>> getUsedJointLimits();

			KDL::Frame getRobotRootWorldTransform();

			bool setFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double> & lower,
					const std::vector<double> & upper);
			//private:
			/****************** Class members ********************/

			/**
			 * \brief Robot Tree Structure
			 */
			std::vector<KinematicElement_t> robot_tree_;    //!< The tree structure
			std::map<std::string, int> segment_map_; //!< Mapping from segment names to positions in the vector robot_tree_
			std::map<std::string, int> joint_map_;
			/**
			 * \brief	ROS/MoveIt SRDF model
			 */
			robot_model::RobotModelPtr model_;
			std::pair<std::string, int> robot_root_;

			std::vector<std::string> used_joints_;
			std::vector<std::string> used_joints_segs_;
			/**
			 * \brief Solver Constants (for optimisation)
			 */
			bool zero_undef_jnts_; //!< Indicates whether we wish to zero-out undefined joints.
			int num_jnts_spec_;	  //!< Number of joints which will be specified
			std::string base_type_;
			std::vector<int> eff_segments_; //!< The End-Effector segments (for Jacobian Computation)
			std::vector<KDL::Frame> eff_seg_offs_; //!< Offsets from the end effector segments (if required)
			std::vector<std::string> eff_segments_ini_;
			std::vector<KDL::Frame> eff_seg_offs_ini_;

			/**
			 *  \brief Solution Storage
			 */
			Eigen::VectorXd forward_map_;	//!< The forward mapping
			Eigen::MatrixXd jacobian_;		//!< The Jacobian
			Eigen::Vector3d com_;	//!< The centre of mass

			/**
			 * \brief Synchronisation and Thread-Safety
			 */
			boost::mutex member_lock_;   //!< Member lock (synchronisation)

			/*************** Private Class Methods *****************/

			/**
			 * \brief Private member function for initialising (wrapper for the buildTree(), setJointOrder() and setEndEffectors() functions)
			 * @param temp_tree     KDL tree from which to construct robot
			 * @param optimisation  Solution Parameters
			 * @return              True if successful, False otherwise
			 */
			bool initialise(const KDL::Tree & temp_tree, const SolutionForm_t & optimisation);

			/**
			 * \brief Builds a robot tree from the kdl tree : NOT THREAD-SAFE
			 * @param temp_tree  The KDL Tree
			 * @param root       Name of the root segment
			 * @param joint_map  The Mapping from the joint to the index for the associated segment
			 * @return           True if ok, false if not
			 */
			bool buildTree(const KDL::Tree & temp_tree, std::string root,
					std::map<std::string, int> & joint_map);

			bool setJointLimits();

			/**
			 * \brief Set the Joint ordering we will use : NOT THREAD-SAFE
			 * @param joints    The list of joints (in order) that will be used
			 * @param zero_out  Indicates whether we will choose to zero out undefined joints
			 * @param joint_map Mapping from joint names to indexes of the correspdonding segment
			 * @return          True if success, false otherwise
			 */
			bool setJointOrder(const std::vector<std::string> & joints, bool zero_out,
					const std::map<std::string, int> & joint_map);

			/**
			 * \brief Set the End Effector parameters : NOT THREAD-SAFE
			 * @param optimisation The Optimisation structure
			 * @return             Indication of success (true) or otherwise
			 */
			bool setEndEffectors(const SolutionForm_t & optimisation);

			/**
			 * \brief Recursive Function which modifies the robot_tree_ and the segment_map_ : NOT THREAD-SAFE
			 * TODO
			 * @return        True if successful, false otherwise
			 */
			bool addSegment(KDL::SegmentMap::const_iterator current_segment, int parent,
					int & current, bool from_ptip, bool to_ctip, const std::string & root_name,
					std::map<std::string, int> & joint_map);

			/**
			 * \brief Recursive function for propagating chains from end-effector to root.
			 * @param	node Index into the next node to process.
			 * @return	 True if successful, false otherwise
			 */
			bool recurseNeedFlag(int node);

			/**
			 * \brief Checks that everything is ok() : NOT THREAD-SAFE
			 * @return	Currently returns the state of the robot_tree_..
			 */
			inline bool isInitialised()
			{
				return robot_tree_.size();
			}
			;

			/**
			 * \brief Private function for computing Forward Mapping : NOT THREAD-SAFE
			 * @return	True if successful, false otherwise
			 */
			bool computePhi();

			/**
			 * \brief Computes the link-specific Positional Jacobian : NOT THREAD-SAFE
			 * @return		True if successful, false otherwise
			 */
			bool computePosJacobian();
	};

	/**
	 * \brief Helper Functions
	 */

	Eigen::Vector3d vectorKdlToEigen(const KDL::Vector & kdl_vec);
	bool xmlGetVector(const tinyxml2::XMLElement & xml_vector, Eigen::VectorXd & eigen_vector);
	bool recursivePrint(kinematica::KinematicTree & robot, std::string node, std::string tab);
}
#endif
