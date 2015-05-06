#include "task_definition/TaskSqrError.h"

REGISTER_TASKDEFINITION_TYPE("TaskSqrError", exotica::TaskSqrError);
REGISTER_TASKDEFINITION_TYPE("TaskVelocitySqrError", exotica::TaskVelocitySqrError);

namespace exotica
{

    TaskVelocitySqrError::TaskVelocitySqrError()
    {
        order=1;
        rho0_.resize(1);
        rho1_.resize(1);
        wasFullyInitialised_ = false;
    }

    TaskSqrError::TaskSqrError()
    {
        order=0;
        rho0_.resize(1);
        rho1_.resize(1);
        wasFullyInitialised_ = false;
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
                    y_star0_.resize(dim);
                    y_star0_.setZero();
                }
                else
                {
                    ERROR("Task definition '"<<object_name_<<"':Goal was not and task map dimension is invalid!");
                    return FAILURE;
                }
            }
        }
        else
        {
            int dim;
            getTaskMap()->taskSpaceDim(dim);
            if(dim>0)
            {
                y_star0_.resize(dim);
                y_star0_.setZero();
            }
            else
            {
                ERROR("Task definition '"<<object_name_<<"':Goal was not and task map dimension is invalid!");
                return FAILURE;
            }
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
        if(wasFullyInitialised_) (*y_star)=(*(y_star_.at(t)));
        y_star_.at(t)=y_star;
        return SUCCESS;
    }


    EReturn TaskSqrError::registerRho(Eigen::VectorXdRef_ptr rho, int t)
    {
        if(wasFullyInitialised_) (*rho)=(*(rho_.at(t)));
        rho_.at(t)=rho;
        return SUCCESS;
    }

    EReturn TaskSqrError::setDefaultGoals(int t)
    {
        if(!wasFullyInitialised_)
        {
            (*(y_star_.at(t)))=y_star0_;
            (*(rho_.at(t)))=rho0_;
        }
        return SUCCESS;
    }

    double TaskSqrError::getRho(int t)
    {
        return (*(rho_.at(t)))(0);
    }

}
