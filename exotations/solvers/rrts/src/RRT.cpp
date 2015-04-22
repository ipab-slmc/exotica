/*
 * RRT.cpp
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */

#include "RRTs/RRT.h"

namespace exotica
{
	RRT::RRT()
	{
		//TODO
	}

	RRT::~RRT()
	{
		//TODO
	}

	EReturn RRT::specifyProblem(PlanningProblem_ptr pointer)
	{
		if (pointer->type().compare(std::string("exotica::SamplingProblem")) != 0)
		{
			ERROR("RRT can't solve problem of type '" << pointer->type() << "'!");
			return PAR_INV;
		}
		problem_ = pointer;
		prob_ = boost::static_pointer_cast<SamplingProblem>(pointer);

		for (auto & it : prob_->getTaskDefinitions())
		{
			if (it.second->type().compare(std::string("exotica::TaskTerminationCriterion")) != 0)
			{
				ERROR("IK Solver currently can only solve exotica::TaskTerminationCriterion. Unable to solve Task: "<<it.second->type());
				return FAILURE;
			}
		}
		return SUCCESS;
	}

	EReturn RRT::initDerived(tinyxml2::XMLHandle & handle)
	{

		return SUCCESS;
	}

	EReturn RRT::Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution)
	{

		return SUCCESS;
	}
}
