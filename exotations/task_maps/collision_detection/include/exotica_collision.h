/*
 * exotica_collision.h
 *
 *  Created on: 8 Aug 2014
 *      Author: yiming
 */

#ifndef EXOTICA_COLLISION_H_
#define EXOTICA_COLLISION_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <kinematica/KinematicTree.h>
#include <Eigen/Eigen>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <fcl/distance.h>
#include "collision_detection_exotica/collision_world_exotica.h"
#include "distance_tools.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace exotica
{
	/**
	 * \brief	Exotica's new collision task
	 */
	class CollisionAvoidance: public TaskMap
	{
		public:
			// Constructor
			CollisionAvoidance();

			// Distructor
			~CollisionAvoidance();

			/**
			 * \brief	Concrete implementation of the update method
			 * @param	x	Robot configuration
			 */
            virtual EReturn update(const Eigen::VectorXd & x, const int t);

			/**
			 * \brief	Get the safe margin
			 * @param	m	Safe margin
			 */
			EReturn getMargin(double m);

			/**
			 * \brief	Set the safe margin
			 * @param	m	Safe margin
			 */
			EReturn setMargin(double m);

			/**
			 * \brief	Get task space dimension
			 */
			EReturn taskSpaceDim(int & task_dim);

			/**
			 * \brief	LAAS specific experiment function
			 * 			Move the world object to base frame
			 * @param	tf	Transform
			 */
			EReturn setObsFrame(const KDL::Frame & tf);
            EReturn setPreUpdateCallback(boost::function<void(CollisionAvoidance*, const Eigen::VectorXd &,int)> pre_update_callback);

            bool publishDebug_;
		protected:
			/**
			 * \brief	Concrete implementation of initialisation from xml
			 * @param	handle	XML handler
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

		private:
			/**
			 * \brief	Compute cost of closest distance
			 * @param	c	The cost of closest distance between robot segments and the world objects
			 */
			EReturn computeCost(double c);

			/**
			 * \brief	Compute the forward map (centre of mass position)
			 * @return	Forward map
			 */
			Eigen::VectorXd computeForwardMap();

			/**
			 * \brief	Compute the jacobian
			 * @param	size		Joint size
			 * @return	Jacobian
			 */
			Eigen::MatrixXd computeJacobian(const int size);

			/**
			 * \brief	Compute the closest distances
			 * @param	x	Robot configuration
			 */
			EReturn computeDistace(const Eigen::VectorXd & x);

			boost::shared_ptr<kinematica::KinematicTree> solver_;	//!< The actual FK Robot solver
			bool initialised_;	//!< Initialisation flag
			boost::mutex lock_;	//!< For thread synchronisation
			double m_;	//!< Safe margin
			std::vector<std::string> links_, joints_;
			std::map<std::string, int> links_map_;
			std::map<std::string, std::vector<int> > eff_map_;
			kinematica::SolutionForm_t initial_sol_;

			exotica::DistanceInfo dist_info_;	//!< Distance information
			std::map<std::string, std::vector<std::string>> acm_;
			std::vector<boost::shared_ptr<fcl::CollisionObject> > dmesh_objs_;

			EParam<std_msgs::String> world_frame_;
			EParam<std_msgs::String> obj_frame_;
			EParam<std_msgs::Bool> dynamic_frame_;
			EParam<std_msgs::Bool> laas_;
            EParam<std_msgs::Bool> useAll_;
			fcl::Transform3f obs_in_base_tf_;
            boost::function<void(CollisionAvoidance*, const Eigen::VectorXd &, int)> pre_update_callback_;

            ros::NodeHandle nh_;
            ros::Publisher state_pub_;
            ros::Publisher wall_pub_;
            visualization_msgs::MarkerArray wall_marker_;
            ros::Publisher close_pub_;
            visualization_msgs::Marker close_;
	};
}
#endif /* EXOTICA_COLLISION_H_ */
