/***********************************************************************\
|    The base class for all Termination Criteria. Since these can be    |
|  quite variable the base implementation only defines the initialisers |
|  and the form of the query function.                                  |
|                                                                       |
|           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
|                    Last Edited: 19 - March - 2014                     |
\***********************************************************************/
#ifndef EXOTICA_TASK_TERMINATION_CRITERION_H
#define EXOTICA_TASK_TERMINATION_CRITERION_H

#include "exotica/TaskDefinition.h"//!< The Component base
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Tools.h"        //!< For XML-Parsing/ErrorFunction definition
#include "task_definition/TaskSqrError.h"

namespace exotica
{
  class TaskTerminationCriterion : public TaskSqrError
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskTerminationCriterion();
      virtual ~TaskTerminationCriterion(){};
      
      

      /**
       * @brief terminate Checks if current state should terminate
       * @param end Returns true if state should terminate
       * @param err Error
       * @return Indication of success
       */
      virtual EReturn terminate(bool & end, double& err, int t=0);

      /**
       * @brief registerGoal Registers threshold reference at time t
       * @param y_star Threshold reference
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerThreshold(Eigen::VectorXdRef_ptr threshold, int t=0);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual EReturn setTimeSteps(const int T);
      

    protected:
      /**
       * \brief Derived-Initialisation
       * @param handle XML handle for any derived parameters
       * @return       Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
      
      /// \brief Threshold on squared error.
      Eigen::VectorXd   threshold0_;
      std::vector<Eigen::VectorXdRef_ptr> threshold_;
  };

  typedef exotica::Factory<std::string, exotica::TaskTerminationCriterion> TerminationCriterionCreator; //!< Convenience name for the EndCriterion Singleton Factory
  typedef boost::shared_ptr<exotica::TaskTerminationCriterion> TaskTerminationCriterion_ptr;
}
#endif
