/*
 * dmesh_ros.h
 *
 *  Created on: 15 Aug 2014
 *      Author: alex
 */

#ifndef DMESH_ROS_H_
#define DMESH_ROS_H_

//EXOTica and SYSTEM packages
#include <exotica/EXOTica.hpp>
#include <tinyxml2/tinyxml2.h>
#include <kinematica/KinematicTree.h>
#include <Eigen/Eigen>
#include <boost/thread/mutex.hpp>
#include "GraphManager.h"
#include <tf/transform_listener.h>
#include <ik_solver/ik_problem.h>
#include <task_definition/TaskSqrError.h>
//ROS packages
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
namespace exotica
{
	/**
	 * \brief	Implementation of distance mesh task map with ROS.
	 * Apart from dMesh task map, this task map use exotica and ROS node, msgs, etc.
	 * The main improvement is now dMeshROS taking computation, visualisation, and robust into consideration
	 * L(d_jl)=K*||p_j-p_l||, K(gain)={PoseGain(kp),ObstacleGain(ko),GoalGain(kg)}
	 */
	class DMeshROS: public TaskMap
	{
		public:
			/**
			 * \brief	Default constructor
			 */
			DMeshROS();

			/**
			 * \brief	Default destructor
			 */
			~DMeshROS();

			/**
			 * \brief	Concrete implementation of update method
			 * @param	x		Joint space configuration
			 */
			virtual EReturn update(const Eigen::VectorXd & x, const int t);

			/**
			 * \brief	Get the task space dimension
			 * @return	Exotica return type, SUCCESS if succeeded
			 */
			virtual EReturn taskSpaceDim(int & task_dim);

			/**
			 * \brief	Get the goal laplace
			 * @param	goal	Goal laplace
			 */
			EReturn getGoalLaplace(Eigen::VectorXd & goal);

			/**
			 * \brief	Update external objects
			 */
			EReturn updateExternal(const exotica::MeshVertex & ext);
			EReturn updateExternal(const exotica::MeshVertexArray & ext);

			EReturn removeVertex(const std::string & name);

			/**
			 * \brief	Get access to the problem
			 * @param	prob	Planning problem
			 */
			EReturn setProblemPtr(const PlanningProblem_ptr & prob);

			/**
			 * \brief	Modify goal on the fly
			 * @param	task_name	Task map name
			 * @param	index	Goal vector entry index
			 * @param	value	New goal value
			 */
			EReturn modifyGoal(const std::string & task_name, const int & index,
					const double & value);

			bool hasActiveObstacle();

			//	Graph Manager
			GraphManager gManager_;
		protected:
			/**
			 * \brief	Concrete implementation of initialisation from xml
			 * @param	handle	XML handler
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

		private:
			/**
			 * \brief	Compute Laplace
			 */
			EReturn computeLaplace();

			/**
			 * \brief	Compute Jacobian
			 */
			EReturn computeJacobian();

			/**
			 * \brief	Update the graph from kinematic scene
			 */
			EReturn updateGraphFromKS();

			/**
			 * \brief	Update the graph externally
			 * @param	name		Vertex name
			 * @param	pose		Vertex position
			 */
			EReturn updateGraphFromExternal(const std::string & name, const Eigen::Vector3d & pose);

			/**
			 * \brief	Update the graph from real transform
			 */
			EReturn updateGraphFromTF();
			/**
			 * \brief	Update the graph from given poses
			 * @param	V		The given links' poses
			 */
			EReturn updateGraphFromPoses(const Eigen::Matrix3Xd & V);

			//	Robot links
			EParam<exotica::StringList> links_;

			//	Link types
			EParam<exotica::BoolList> link_types_;

			//	If we want to get real joint state
			tf::TransformListener listener_;

			tf::StampedTransform transform_;

			//	Maximum graph size
			EParam<std_msgs::Int64> size_;

			//	Robot links size
			int robot_size_;

			//	External objects size
			int ext_size_;

			//	Task space size
			int task_size_;

			//	Configuration size
			int q_size_;

			//	Gain to keep robot pose
			EParam<std_msgs::Float64> kp_;

			//	Gain to avoid obstacle
			EParam<std_msgs::Float64> ko_;

			//	Gain to reach goal
			EParam<std_msgs::Float64> kg_;

			//	Laplace
			Eigen::VectorXd laplace_;

			//	Jacobian
			Eigen::MatrixXd jac_;

			//	Distance matrix
			Eigen::MatrixXd dist_;

			//	Initialisation flag
			bool initialised_;

			//	Scoped locker
			boost::mutex::scoped_lock lock_;

			//	Pointer to the problem
			IKProblem_ptr prob_;

			//	True if the obstacle is close
			std::vector<bool> obs_close_;

			//	Interact Range
			double ir_;

			double wo_;
			double wg_;

			EParam<std_msgs::Bool> usePose_;

	};
	typedef boost::shared_ptr<DMeshROS> DMeshROS_Ptr;
} //namespace exotica

#endif /* DMESH_ROS_H_ */
