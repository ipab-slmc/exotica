#include "task_definition/TaskTerminationCriterion.h"

exotica::TaskTerminationCriterion::TaskTerminationCriterion()
{
	status_ = CONTINUE;
}

exotica::TaskTerminationCriterion::~TaskTerminationCriterion()
{

}

exotica::EReturn exotica::TaskTerminationCriterion::initDerived(tinyxml2::XMLHandle & handle)
{

	if (handle.FirstChildElement("Goal").ToElement())
	{
		Eigen::VectorXd goal;
		if(!ok(getVector(*(handle.FirstChildElement("Goal").ToElement()), goal)))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (!ok(setGoal(goal)))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
	}
	else
	{
		INDICATE_FAILURE
		return FAILURE;
	}
	return SUCCESS;
}

