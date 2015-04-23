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

	OMPLProblem::OMPLProblem()
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
		return SUCCESS;
	}

	int OMPLProblem::getSpaceDim()
	{
		int n = 0;
		for (auto scene : scenes_)
		{
			int nn = scene.second->getNumJoints();
			if (n == 0)
			{
				n = nn;
				continue;
			}
			else
			{
				if (n != nn)
				{
					ERROR("Kinematic scenes have different joint space sizes!");
					return -1;
				}
				else
				{
					continue;
				}
			}
		}
		return n;
	}

	std::vector<TaskTerminationCriterion_ptr>& OMPLProblem::getGoals()
	{
		return goals_;
	}

} /* namespace exotica */
