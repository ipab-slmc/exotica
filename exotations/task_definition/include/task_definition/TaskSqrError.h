/*!*******************************************************************!*\
|    TaskSqrError provides storage for squared-error forms of Task      |
|  definitions. It attempts to implement this through a thread-safe     |
|  accessor system.                                                     |
|                                                                       |
|           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
|                    Last Edited: 31 - March - 2014                     |
\***********************************************************************/

#ifndef EXOTICA_ERROR_CLASS_H
#define EXOTICA_ERROR_CLASS_H

#include "exotica/TaskDefinition.h"//!< The Component base
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Tools.h"        //!< For XML-Parsing/ErrorFunction definition
#include "exotica/Test.h"         //!< For Testing factory
#include <Eigen/Dense>            //!< Generally dense manipulations should be enough
#include <boost/thread/mutex.hpp> //!< The boost thread-library for synchronisation
 
namespace exotica
{
  class TaskSqrError : public TaskDefinition
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskSqrError();
      virtual ~TaskSqrError(){}
      
      /**
       * @brief registerGoal Registers a goal reference at time t
       * @param y_star Goal reference
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerGoal(Eigen::VectorXdRef_ptr y_star, int t=0);

      /**
       * @brief registerGoal Registers rho reference at time t
       * @param y_star Rho reference
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerRho(Eigen::VectorXdRef_ptr rho, int t=0);

      /**
       * @brief getRho Returns the value of rho at time step t
       * @param t Timestep
       * @return rho
       */
      double getRho(int t);

      /**
       * @brief setRho Returns the value of rho at time step t
       * @param t Timestep
       * @param rho
       */
      EReturn setRho(int t, double rho);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual EReturn setTimeSteps(const int T);
      int getTimeSteps(){return y_star_.size();}

      /**
       * @brief setDefaultGoals Sets Goals and Rhos to default values
       * @return Indicates success
       */
      EReturn setDefaultGoals(int t);

      Eigen::VectorXd   y_star0_;    //!< The goal vector
      Eigen::VectorXd   rho0_,rho1_;       //!< The scalar inter-task weight
      bool wasFullyInitialised_;

      /**
       * \bref	Get goal
       * @param	t		Time step
       * @return		Goal
       */
      Eigen::VectorXdRef_ptr getGoal(int t = 0);

    protected:
      /**
       * \brief Concrete implementation of the initDerived
       * @param handle  The handle to the XML-element describing the ErrorFunction Function
       * @return        Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
      
      /** The internal storage **/
      std::vector<Eigen::VectorXdRef_ptr>   y_star_;    //!< The goal vector
      std::vector<Eigen::VectorXdRef_ptr>   rho_;       //!< The scalar inter-task weight




  };
  typedef boost::shared_ptr<TaskSqrError> TaskSqrError_ptr;
  class TaskVelocitySqrError : public TaskSqrError
  {
  public:
    TaskVelocitySqrError();
  };
  typedef boost::shared_ptr<TaskSqrError> TaskSqrError_ptr;
  typedef boost::shared_ptr<TaskVelocitySqrError> TaskVelocitySqrError_ptr;
}
#endif
