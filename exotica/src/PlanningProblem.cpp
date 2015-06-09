#include "exotica/PlanningProblem.h"

namespace exotica
{
	PlanningProblem::PlanningProblem()
	{

	}

    EReturn PlanningProblem::reinitialise(rapidjson::Document& document, boost::shared_ptr<PlanningProblem> problem)
    {
        ERROR("This has to implemented in the derived class!");
        return FAILURE;
    }

    std::string PlanningProblem::print(std::string prepend)
    {
        std::string ret = Object::print(prepend);
        ret+="\n"+prepend+"  Task definitions:";
        for(auto& it : task_defs_) ret+="\n"+it.second->print(prepend+"    ");
        return ret;
    }

    EReturn PlanningProblem::initBase(tinyxml2::XMLHandle & handle,
            const Server_ptr & server)
    {
        poses.reset(new std::map<std::string,Eigen::VectorXd>());
        posesJointNames.reset(new std::vector<std::string>());
        knownMaps_["PositionConstraint"]="Distance";
        knownMaps_["QuatConstraint"]="Orientation";
        knownMaps_["PostureConstraint"]="Identity";

        startState.resize(0);
        endState.resize(0);
        nominalState.resize(0);

		Object::initBase(handle, server);
		if (!server)
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		server_ = server;
		//!< Temporaries
		tinyxml2::XMLHandle xml_handle(handle);
		EReturn ret_value = SUCCESS;
		int count;
		std::string name;
		std::string type;

		//!< Refresh
		scenes_.clear();
		task_maps_.clear();
		task_defs_.clear();

		//!< First create the Kinematic Scenes
		xml_handle = handle.FirstChildElement("Scene");
		count = 0;
		while (xml_handle.ToElement() and ok(ret_value)) //!< While we are still in a valid situation
		{
			const char * temp_name = xml_handle.ToElement()->Attribute("name");
			if (temp_name == nullptr)
			{
				INDICATE_FAILURE
				ret_value = PAR_ERR;
				break;
			}
			name = temp_name;
			if (scenes_.find(name) != scenes_.end())
			{
				INDICATE_FAILURE
				ret_value = PAR_ERR;
				break;
			}
			scenes_[name].reset(new Scene(name));
			if (scenes_[name] == nullptr)
			{
				INDICATE_FAILURE
				ret_value = PAR_ERR;
				break;
			}
			scenes_.at(name)->initialisation(xml_handle, server_);
			count++;
			xml_handle = xml_handle.NextSiblingElement("Scene");
		}

		//!< No maps defined:
		if (count < 1 and ok(ret_value))
		{
			ret_value = WARNING;
		}

		//!< Now we will create the maps
		xml_handle = handle.FirstChildElement("Map");
		count = 0;
		while (xml_handle.ToElement() and ok(ret_value)) //!< While we are still in a valid situation
		{
			const char * temp_name = xml_handle.ToElement()->Attribute("name");
			if (temp_name == nullptr)
			{
				INDICATE_FAILURE
				;
				ret_value = PAR_ERR;
				break;
			}
			name = temp_name;
			if (task_maps_.find(name) != task_maps_.end())
			{
				INDICATE_FAILURE
				;
				ret_value = PAR_ERR;
				break;
			}
			const char * temp_type = xml_handle.ToElement()->Attribute("type");
			if (temp_type == nullptr)
			{
				INDICATE_FAILURE
				;
				ret_value = PAR_ERR;
				break;
			}
			type = temp_type;
			TaskMap_ptr temp_ptr;
			EReturn aux_rtn = TaskMap_fac::Instance().createObject(type, temp_ptr);
			if (ok(aux_rtn))
			{
				ret_value = aux_rtn;
			}
			if (!ok(ret_value))
			{
				ERROR("Can not create task map of type '" << type << "'");
			}
			if (ok(ret_value))
			{
				task_maps_[name] = temp_ptr;  //!< Copy the shared_ptr;
				task_maps_.at(name)->ns_ = ns_ + "/" + name;
				count++;
				aux_rtn = temp_ptr->initBase(xml_handle, server_, scenes_);
				if (ok(aux_rtn))
				{
					ret_value = aux_rtn;
				}
				if (!ok(ret_value))
				{
					INDICATE_FAILURE
					;
				}
			}
			xml_handle = xml_handle.NextSiblingElement("Map");
		}
		//!< No maps defined:
		if (count < 1 and ok(ret_value))
		{
			ret_value = WARNING;
		}

		//!< NEW------------
		//!< Now we initialise the scene
		for (auto & it : scenes_)
		{
			if (!ok(it.second->activateTaskMaps()))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}

		//!< Now the Task Definitions (all)
		xml_handle = handle.FirstChildElement("Task");
		count = 0;
		while (xml_handle.ToElement() and ok(ret_value))  //!< May not be ok due to previous error
		{
			//!< Check that name is available and not duplicated
			const char * temp_name = xml_handle.ToElement()->Attribute("name");
			if (temp_name == nullptr)
			{
				INDICATE_FAILURE
				;
				ret_value = PAR_ERR;
				break;
			}
			name = temp_name;
			if (task_defs_.find(name) != task_defs_.end())
			{
				INDICATE_FAILURE
				;
				ret_value = PAR_ERR;
				break;
			}

			//!< Check that Type is also available
			const char * temp_type = xml_handle.ToElement()->Attribute("type");
			if (temp_type == nullptr)
			{
				INDICATE_FAILURE
				;
				ret_value = PAR_ERR;
				break;
			}
			type = temp_type;

			//!< attempt to create
			TaskDefinition_ptr temp_ptr;
			EReturn aux_rtn = TaskDefinition_fac::Instance().createObject(type, temp_ptr);
			if (aux_rtn)
			{
				ret_value = aux_rtn;
			}
			if (!ok(ret_value))
			{
				INDICATE_FAILURE
			}

			//!< Attempt to initialise
			if (ok(ret_value))
			{
				task_defs_[name] = temp_ptr;
				task_defs_.at(name)->ns_ = ns_ + "/" + name;
				aux_rtn = temp_ptr->initBase(xml_handle, task_maps_);
				if (aux_rtn)
				{
					ret_value = aux_rtn;
				}
				if (!ok(ret_value))
				{
					INDICATE_FAILURE
					break;
				}
			}

			//!< Prepare for next iteration (if made it this far)
			count++;
			xml_handle = xml_handle.NextSiblingElement("Task");
		}
		//!< IF no task definitions defined
		if (count < 1 and ok(ret_value))
		{
			ret_value = WARNING;
		}

		//!< If ok so far...
		if (ok(ret_value))
		{
			EReturn temp_return = initDerived(handle);
			if (temp_return)
			{
				ret_value = temp_return;
			}
			if (!ok(ret_value))
			{
				INDICATE_FAILURE
			}
		}
		//!< Resolve if not ok to refresh
		if (!ok(ret_value))
		{
			scenes_.clear();
			task_maps_.clear();
			task_defs_.clear();
		}
		//!< Exit
		if (!server)
			return FAILURE;

        originalMaps_=task_maps_;
        originalDefs_=task_defs_;

		return ok(ret_value) ? initDerived(handle) : ret_value;
	}

    void PlanningProblem::clear(bool keepOriginals)
    {
        if(keepOriginals)
        {
            task_maps_=originalMaps_;
            task_defs_=originalDefs_;
        }
        else
        {
            task_maps_.clear();
            task_defs_.clear();
        }
    }

	EReturn PlanningProblem::update(Eigen::VectorXdRefConst x, const int t)
	{
		// Update the KinematicScene(s)...
		for (auto it = scenes_.begin(); it != scenes_.end(); ++it)
		{
			if (!ok(it->second->update(x)))
			{
				INDICATE_FAILURE
				;
				return FAILURE;
			}
		}

		// Update the Task maps

#ifdef EXOTICA_DEBUG_MODE
		if (!((x - x).array() == (x - x).array()).all())
		{
			ROS_ERROR_STREAM("Infinite q= "<<x.transpose());
			INDICATE_FAILURE
			return FAILURE;
		}
#endif
		for (TaskMap_map::const_iterator it = task_maps_.begin(); it != task_maps_.end(); ++it)
		{
			if (!ok(it->second->update(x, t)))
			{
				INDICATE_FAILURE
				return FAILURE;
			}

		}

		return SUCCESS;
	}

	TaskDefinition_map& PlanningProblem::getTaskDefinitions()
	{
		return task_defs_;
	}

	TaskMap_map& PlanningProblem::getTaskMaps()
	{
		return task_maps_;
	}

	Scene_map& PlanningProblem::getScenes()
	{
		return scenes_;
	}

	EReturn PlanningProblem::setScene(const planning_scene::PlanningSceneConstPtr & scene)
	{
		for (auto & it : scenes_)
		{
			if (!ok(it.second->setCollisionScene(scene)))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}
		return SUCCESS;
	}
	EReturn PlanningProblem::setScene(const moveit_msgs::PlanningSceneConstPtr & scene)
	{
		for (auto & it : scenes_)
		{
			if (!ok(it.second->setCollisionScene(scene)))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}
		return SUCCESS;
	}

	EReturn PlanningProblem::updateKinematicScene(
			const planning_scene::PlanningSceneConstPtr & scene)
	{
		INDICATE_FAILURE
		ROS_ERROR("Kinematica Scene is duplicated in new EXOTica 3.5, you should not call this function");
		return FAILURE;
	}
}
