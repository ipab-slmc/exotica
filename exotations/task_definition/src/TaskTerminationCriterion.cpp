#include "task_definition/TaskTerminationCriterion.h"

REGISTER_TASKDEFINITION_TYPE("TaskTerminationCriterion",
    exotica::TaskTerminationCriterion);

namespace exotica
{

  TaskTerminationCriterion::TaskTerminationCriterion()
      : threshold_(0.0)
  {
    order = 0;
    rho0_.resize(1);
    rho1_.resize(1);
    threshold0_.resize(1);
    wasFullyInitialised_ = false;
  }

  EReturn TaskTerminationCriterion::initDerived(tinyxml2::XMLHandle & handle)
  {
    if (ok(TaskSqrError::initDerived(handle)))
    {
      double thr;
      if (handle.FirstChildElement("Threshold").ToElement())
      {
        if (ok(
            getDouble(*(handle.FirstChildElement("Threshold").ToElement()),
                thr)))
        {
          threshold0_(0) = thr;
        }
        else
        {
          INDICATE_FAILURE
          ;
          return PAR_ERR;
        }
      }
      else
      {
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }

    setTimeSteps(1);

    return SUCCESS;
  }

  EReturn TaskTerminationCriterion::terminate(bool & end, double& err, int t)
  {
    err = ((*(task_map_->phi_.at(t))) - (*(y_star_.at(t)))).squaredNorm()
        * (*(rho_.at(t)))(0);
    end = err <= (*(threshold_.at(t)))(0);
//    	HIGHLIGHT_NAMED(object_name_,"Phi "<<task_map_->phi_.at(t)->transpose()<<" goal "<<y_star_.at(t)->transpose()<<" Err "<<err);
    return SUCCESS;
  }

  EReturn TaskTerminationCriterion::registerThreshold(
      Eigen::VectorXdRef_ptr threshold, int t)
  {
    threshold_.at(t) = threshold;
    return SUCCESS;
  }

  EReturn TaskTerminationCriterion::setTimeSteps(const int T)
  {
    TaskSqrError::setTimeSteps(T);
    threshold_.assign(T, Eigen::VectorXdRef_ptr(threshold0_));
    return SUCCESS;
  }

}
