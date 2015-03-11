/*
 * Scene.h
 *
 *  Created on: 10 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_
#define EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_

#include "exotica/Object.h"
#include "exotica/Server.h"
#include "tinyxml2/tinyxml2.h"
#include <kinematica/KinematicTree.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>

namespace exotica
{
	class Scene
	{
		public:
			/*
			 * \brief	Default constructor
			 * @param	name	The scene name
			 */
			Scene(const std::string & name);

			/*
			 * \brief	Destructor
			 */
			virtual ~Scene();

			/*
			 * \brief	Get scene name
			 * @return	Name
			 */
			std::string getName();

			/*
			 * \brief	Initialisation function
			 * @param	handle	XML handle
			 * @param	server	Server pointer
			 */
			EReturn initialisation(tinyxml2::XMLHandle & handle, const Server_ptr & server);

			/*
			 * \brief	Updator function
			 * @param	x	System state
			 */
			virtual EReturn update(const Eigen::VectorXd x, const int t = 0);

			/*
			 * \brief	Append new taskmap
			 * @param	name	Taskmap name
			 * @param	eff		Endeffector names
			 * @param	offset	Endeffector offsets
			 */
			EReturn appendTaskMap(const std::string & name, const std::vector<std::string> & eff,
					const std::vector<KDL::Frame> & offset);

			/*
			 * \brief	Called after appending
			 */
			EReturn activateTaskMaps();

			/*
			 * \brief	Update task map (eff)
			 * @param	task	Task name
			 * @param	offset	Task end-effector offsets
			 */
			EReturn updateEndEffectors(const std::string & task,
					const std::vector<KDL::Frame> & offset);

			/*
			 * \breif	Get forward map
			 * @param	task	Task name
			 * @param	phi		Forward map
			 */
			EReturn getForwardMap(const std::string & task, Eigen::Ref<Eigen::VectorXd> phi);

			/*
			 * \brief	Get jacobian
			 * @param	task	Task name
			 * @param	jac		Jacobian
			 */
			EReturn getJacobian(const std::string & task, Eigen::Ref<Eigen::MatrixXd> jac);

			/*
			 * \brief	Get joint number N
			 * @return	N
			 */
			int getNumJoints();

			/*
			 * \bref	Get map size for particular taskmap
			 * @param	task	Task name
			 * @return	Map 	Size
			 */
			int getMapSize(const std::string & task);

			/**
			 * \brief	Get centre of mass properties
			 * @param	seg		Segment names
			 * @param	mass	Mass of each link
			 * @param	cog		Centre of gravity of each link
			 * @param	tip_pose	Tip pose of each link
			 * @param	base_pose	Base pose of each link
			 */
			EReturn getCoMProperties(std::vector<std::string> & segs, Eigen::VectorXd & mass,
					std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
					std::vector<KDL::Frame> & base_pose);

			/*
			 * \brief	Get task root name
			 * @return	Root name
			 */
			std::string getRootName();

			planning_scene::PlanningScenePtr getPlanningScene();
			kinematica::KinematicTree & getSolver();
		private:
			//	The name of the scene
			std::string name_;

			//	The kinematica tree
			kinematica::KinematicTree kinematica_;

			//	The controlled joint size
			int N;

			Eigen::VectorXd Phi_;

			Eigen::MatrixXd Jac_;

			//	Forwardmaps
			std::map<std::string, boost::shared_ptr<Eigen::Ref<Eigen::VectorXd> > > phis_;

			//	Jacobians
			std::map<std::string, boost::shared_ptr<Eigen::Ref<Eigen::MatrixXd> > > jacs_;

			//	End-effector names
			std::map<std::string, std::vector<std::string> > eff_names_;

			//	End-effector offsets
			std::map<std::string, std::vector<KDL::Frame> > eff_offsets_;

			//	End-effector index (in kinematica)
			std::map<std::string, std::vector<int> > eff_index_;

			boost::mutex lock_;
	};
	typedef boost::shared_ptr<Scene> Scene_ptr;
	typedef std::map<std::string, Scene_ptr> Scene_map;
}	//	namespace exotica

#endif /* EXOTICA_EXOTICA_INCLUDE_EXOTICA_SCENE_H_ */
