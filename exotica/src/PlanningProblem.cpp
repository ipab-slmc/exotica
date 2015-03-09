#include "exotica/PlanningProblem.h"

exotica::PlanningProblem::PlanningProblem()
{
}

exotica::EReturn exotica::PlanningProblem::initBase(tinyxml2::XMLHandle & handle,
        const Server_ptr & server)
{
    Object::initBase(handle,server);
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
	k_scenes_.clear();
	task_maps_.clear();
	task_defs_.clear();

	//!< First create the Kinematic Scenes
	xml_handle = handle.FirstChildElement("KScene");
	count = 0;
	while (xml_handle.ToElement() and ok(ret_value))  //!< While we are still in a valid situation
	{
		const char * temp_name = xml_handle.ToElement()->Attribute("name");
		if (temp_name == nullptr)
		{
			INDICATE_FAILURE ret_value = PAR_ERR;
			break;
		}
		name = temp_name;
		if (k_scenes_.find(name) != k_scenes_.end())
		{
			INDICATE_FAILURE ret_value = PAR_ERR;
			break;
		}
		k_scenes_[name].reset(new kinematica::KinematicScene(name));
		if (k_scenes_[name] == nullptr)
		{
			INDICATE_FAILURE ret_value = PAR_ERR;
			break;
		}
		if (!k_scenes_[name]->initialised_)
		{
			INDICATE_FAILURE ret_value = FAILURE;
			break;
		}
		if (!k_scenes_[name]->initKinematicScene(xml_handle))
		{
			INDICATE_FAILURE ret_value = FAILURE;
			break;
		}
		xml_handle = xml_handle.NextSiblingElement("KScene");
	}
	//!< No maps defined:
	if (count < 1 and ok(ret_value))
	{
		ret_value = WARNING;
	}

	//!< Now we will create the maps
	xml_handle = handle.FirstChildElement("Map");
	count = 0;
	while (xml_handle.ToElement() and ok(ret_value))  //!< While we are still in a valid situation
	{
		const char * temp_name = xml_handle.ToElement()->Attribute("name");
		if (temp_name == nullptr)
		{
			INDICATE_FAILURE;
			ret_value = PAR_ERR;
			break;
		}
		name = temp_name;
		if (task_maps_.find(name) != task_maps_.end())
		{
			INDICATE_FAILURE;
			ret_value = PAR_ERR;
			break;
		}
		const char * temp_type = xml_handle.ToElement()->Attribute("type");
		if (temp_type == nullptr)
		{
			INDICATE_FAILURE;
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
			aux_rtn = temp_ptr->initBase(xml_handle, server_, k_scenes_);
			if (ok(aux_rtn))
			{
				ret_value = aux_rtn;
			}
			if (!ok(ret_value))
			{
				INDICATE_FAILURE;
			}
		}
		xml_handle = xml_handle.NextSiblingElement("Map");
	}
	//!< No maps defined:
	if (count < 1 and ok(ret_value))
	{
		ret_value = WARNING;
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
			INDICATE_FAILURE;
			ret_value = PAR_ERR;
			break;
		}
		name = temp_name;
		if (task_defs_.find(name) != task_defs_.end())
		{
			INDICATE_FAILURE;
			ret_value = PAR_ERR;
			break;
		}

		//!< Check that Type is also available
		const char * temp_type = xml_handle.ToElement()->Attribute("type");
		if (temp_type == nullptr)
		{
			INDICATE_FAILURE;
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
		k_scenes_.clear();
		task_maps_.clear();
		task_defs_.clear();
	}
	//!< Exit
	if (!server)
		return FAILURE;
	return ok(ret_value) ? initDerived(handle) : ret_value;
}

exotica::EReturn exotica::PlanningProblem::update(const Eigen::VectorXd & x, const int t)
{
	EReturn ret_value = SUCCESS;

	//!< Update the KinematicScene(s)...
	for (auto it = k_scenes_.begin(); it != k_scenes_.end() and ok(ret_value); ++it)
	{
        ret_value = it->second->update(x, t) ? SUCCESS : FAILURE;
	}

	//!< Update the Task maps
	if (task_maps_.size() == 0 and ok(ret_value))
	{
		ret_value = WARNING;
	}
	else
	{
		for (TaskMap_map::const_iterator it = task_maps_.begin();
				it != task_maps_.end() and ok(ret_value); ++it)
		{
            EReturn temp_return = it->second->update(x,t);
			if (temp_return)
			{
				ret_value = temp_return;
			}
		}
	}

	return ret_value;
}

exotica::EReturn exotica::PlanningProblem::updateKinematicScene(
		const planning_scene::PlanningSceneConstPtr & scene)
{
	std::map<std::string, kinematica::KinematicScene_ptr>::iterator it;
	for (it = k_scenes_.begin(); it != k_scenes_.end(); ++it)
	{
		if (!it->second->updateScene(scene))
			return FAILURE;
	}
	return SUCCESS;
}
