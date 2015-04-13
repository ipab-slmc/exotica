#include "task_definition/TaskTerminationCriterion.h"

REGISTER_TASKDEFINITION_TYPE("TaskTerminationCriterion", exotica::TaskTerminationCriterion);

namespace exotica
{

	TaskTerminationCriterion::TaskTerminationCriterion() :
			threshold_(0.0),
			dim_(0)
	{

	}

	EReturn TaskTerminationCriterion::initDerived(tinyxml2::XMLHandle & handle)
	{
		EReturn temp_return = TaskSqrError::initDerived(handle);
		double thr;
		if (temp_return) { INDICATE_FAILURE; }
		else
											{ if (!handle.FirstChildElement("Threshold").ToElement()) {temp_return = PAR_ERR; } }
		if (temp_return) { INDICATE_FAILURE; }
		else             { temp_return = getDouble(*(handle.FirstChildElement("Threshold").ToElement()), thr); }
		if (temp_return) { INDICATE_FAILURE; }
		else             { temp_return = setThreshold(thr); }
		taskSpaceDim(dim_);
		y_.resize(dim_);
		return temp_return;
	}

	EReturn TaskTerminationCriterion::terminate(bool & end, double& err)
	{
		phi(y_);
		err=(y_-y_star_).squaredNorm()*rho_;
		end = err<threshold_;
		return SUCCESS;
	}

	EReturn TaskTerminationCriterion::setThreshold(const double & thr)
	{
		threshold_=thr;
		return SUCCESS;
	}

	EReturn TaskTerminationCriterion::getThreshold(double & thr)
	{
		thr=threshold_;
		return SUCCESS;
	}

}
