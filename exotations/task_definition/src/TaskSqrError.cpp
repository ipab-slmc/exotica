#include "task_definition/TaskSqrError.h"

REGISTER_TASKDEFINITION_TYPE("TaskSqrError", exotica::TaskSqrError);
REGISTER_TASKDEFINITION_TYPE("TaskVelocitySqrError", exotica::TaskVelocitySqrError);

namespace exotica
{

    TaskVelocitySqrError::TaskVelocitySqrError()
    {
        order=1;
    }

    TaskSqrError::TaskSqrError()
    {
        order=0;
    }

    EReturn TaskSqrError::initDerived(tinyxml2::XMLHandle & handle)
    {
        //!< Temporaries
        Eigen::VectorXd y_star; //!< The Goal vector
        double          rho;

        // Load Rho
        if (handle.FirstChildElement("Rho").ToElement())
        {
          if(ok(getDouble(*(handle.FirstChildElement("Rho").ToElement()), rho)))
          {
              rho0_.resize(1);
              rho0_(0)=rho;
          }
          else
          {
              INDICATE_FAILURE;
              return PAR_ERR;
          }
        }
        else
        {
          INDICATE_FAILURE;
          return PAR_ERR;
        }

        // Load the goal
        if (handle.FirstChildElement("Goal").ToElement())
        {
            if(ok(getVector(*(handle.FirstChildElement("Goal").ToElement()), y_star)))
            {
                y_star0_=y_star;
            }
            else
            {
                int dim;
                getTaskMap()->taskSpaceDim(dim);
                if(dim>0)
                {
                    y_star0_.resize(0);
                    y_star0_.setZero();
                    WARNING("Task definition '"<<object_name_<<"':Goal was not specified, setting goal to zeros.");
                }
                else
                {
                    WARNING("Task definition '"<<object_name_<<"':Goal was not specified and task vector size is dynamic.");
                }
            }
        }
        else
        {
          INDICATE_FAILURE;
          return PAR_ERR;
        }

        // Set default number of time steps
        setTimeSteps(1);

        return SUCCESS;
    }

    EReturn TaskSqrError::setTimeSteps(const int T)
    {
        TaskDefinition::setTimeSteps(T);
        y_star_.assign(T,Eigen::VectorXdRef_ptr(y_star0_));
        rho_.assign(T,Eigen::VectorXdRef_ptr(rho0_));
        return SUCCESS;
    }

    EReturn TaskSqrError::registerGoal(Eigen::VectorXdRef_ptr y_star, int t)
    {
        y_star_.at(t)=y_star;
        return SUCCESS;
    }


    EReturn TaskSqrError::registerRho(Eigen::VectorXdRef_ptr rho, int t)
    {
        rho_.at(t)=rho;
        return SUCCESS;
    }

    EReturn TaskSqrError::modifyGoal(const int & index, const double & value, int t)
    {
        //	Modifying goal on the fly, ignore it unless you using DMesh
        (*(y_star_.at(t)))(index) = value;
        return SUCCESS;
    }

    EReturn TaskSqrError::setDefaultGoals(int t)
    {
        (*(y_star_.at(t)))=y_star0_;
        (*(rho_.at(t)))=rho0_;
        return SUCCESS;
    }

    double TaskSqrError::getRho(int t)
    {
        return (*(rho_.at(t)))(0);
    }

}
