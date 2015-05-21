/*
 * OMPLProblem.cpp
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#include "ompl_solver/OMPLProblem.h"

REGISTER_PROBLEM_TYPE("OMPLProblem", exotica::OMPLProblem);

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}

namespace exotica
{

	OMPLProblem::OMPLProblem() :
			space_dim_(0)
	{
		// TODO Auto-generated constructor stub

	}

	OMPLProblem::~OMPLProblem()
	{
		// TODO Auto-generated destructor stub
	}

	std::vector<double>& OMPLProblem::getBounds()
	{
		return bounds_;
	}

	EReturn OMPLProblem::initDerived(tinyxml2::XMLHandle & handle)
	{
		for (auto goal : task_defs_)
		{
			if (goal.second->type().compare("exotica::TaskTerminationCriterion") == 0)
			{
				goals_.push_back(boost::static_pointer_cast<exotica::TaskTerminationCriterion>(goal.second));
			}
		}
		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("LocalPlannerConfig");
		if (tmp_handle.ToElement())
		{
			local_planner_config_ = tmp_handle.ToElement()->GetText();
		}

		for (auto scene : scenes_)
		{
			int nn = scene.second->getNumJoints();
			if (space_dim_ == 0)
			{
				space_dim_ = nn;
				continue;
			}
			else
			{
				if (space_dim_ != nn)
				{
					ERROR("Kinematic scenes have different joint space sizes!");
					return FAILURE;
				}
				else
				{
					continue;
				}
			}
		}

		std::vector<std::string> jnts;
		scenes_.begin()->second->getJointNames(jnts);
		robot_model::RobotModelConstPtr model=server_->getModel("robot_description");
		getBounds().resize(jnts.size() * 2);
		for (int i = 0; i < jnts.size(); i++)
		{
			getBounds()[i] = model->getJointModel(jnts[i])->getVariableBounds()[0].min_position_;
			getBounds()[i + jnts.size()] = model->getJointModel(jnts[i])->getVariableBounds()[0].max_position_;
		}
		return SUCCESS;
	}

	int OMPLProblem::getSpaceDim()
	{
		return space_dim_;
	}

	std::vector<TaskTerminationCriterion_ptr>& OMPLProblem::getGoals()
	{
		return goals_;
	}

} /* namespace exotica */
