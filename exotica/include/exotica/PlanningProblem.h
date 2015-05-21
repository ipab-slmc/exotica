/***********************************************************************\
|    This class defines the over-arching problem specification for      |
|  EXOTica. It is implemented in an abstract fashion, which can be      |
|  specialised for the various types by derived instances.              |
|                                                                       |
|           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
|                    Last Edited: 24 - March - 2014                     |
\***********************************************************************/
#ifndef EXOTICA_MOTION_PLANNING_PROBLEM_H
#define EXOTICA_MOTION_PLANNING_PROBLEM_H

#include "exotica/Object.h"
#include "exotica/TaskMap.h"
#include "exotica/TaskDefinition.h"
#include "exotica/Server.h"
#include "exotica/Scene.h"
#include "exotica/Tools.h"
#include "tinyxml2/tinyxml2.h"

#include <vector>
#include <string>
#include <map>

#define REGISTER_PROBLEM_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::PlanningProblem, TYPE, DERIV)

namespace exotica
{
	class PlanningProblem: public Object
	{
		public:
			/**
			 * \brief Default Constructor
			 */
			PlanningProblem();
			virtual ~PlanningProblem(){};

			/**
			 * \brief Initialiser (from XML): takes care of instantiating the TaskMaps and Definitions and the Kinematic Scenes
			 * @param handle[in] The handle to the XML-element describing the Problem Definition
			 * @param	server	Server
			 * @return           Indication of success/failure: TODO
			 */
			EReturn initBase(tinyxml2::XMLHandle & handle, const Server_ptr & server);

			/**
			 * \brief Updator: declared virtual so can be overridden.
			 * @param x The state of the system
             * @param t Time step (not used by most task maps)
			 * @return  Indication of success TODO
			 */
            virtual EReturn update(Eigen::VectorXdRefConst x, const int t);

			/**
			 * \brief Returns the reference to the task definition map.
			 * @return Task definitions
			 */
			TaskDefinition_map& getTaskDefinitions();

			/**
			 * \brief	Get task maps
			 * @return Task maps
			 */
			TaskMap_map& getTaskMaps();

            Scene_map& getScenes();

			/**
			 * \brief Update the kinematic scene
			 * @param scene	The planning scene from moveit
			 */
			EReturn updateKinematicScene(const planning_scene::PlanningSceneConstPtr & scene);

			/*
			 * \brief	Set EXOTica Scene from a moveit planning scene
			 * @param	scene	Moveit planning scene
			 */
			EReturn setScene(const planning_scene::PlanningSceneConstPtr & scene);
			EReturn setScene(const moveit_msgs::PlanningSceneConstPtr & scene);
            Scene_map scenes_;  //!< Kinematic scene(s) indexed by name

            Eigen::VectorXd startState;

            virtual EReturn reinitialise(rapidjson::Document& document, boost::shared_ptr<PlanningProblem> problem);

            boost::shared_ptr<std::map<std::string,Eigen::VectorXd> > poses;
            boost::shared_ptr<std::vector<std::string> > posesJointNames;

		protected:

			/**
			 * \brief Derived Initialiser (from XML): PURE VIRTUAL
			 * @param handle[in] The handle to the XML-element describing the Problem Definition
			 * @return           Indication of success/failure: TODO
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle) = 0;

			Server_ptr server_; //!< Pointer to EXOTica parameter server;
			TaskMap_map task_maps_; //!< The set of taskmaps we will be using, which will be shared between task-definitions
			TaskDefinition_map task_defs_; //!< The set of task definition objects
            std::map<std::string,std::string> knownMaps_;

	};
	
	typedef Factory<std::string, PlanningProblem> PlanningProblem_fac;
	typedef boost::shared_ptr<PlanningProblem>    PlanningProblem_ptr;
	
	
	
}
#endif
