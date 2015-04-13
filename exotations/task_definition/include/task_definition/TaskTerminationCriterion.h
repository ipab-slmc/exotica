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
       * \brief Terminate Query: PURE VIRTUAL
       * @pre                  Should check initialisation is ok
       * @param terminate[out] Returns indication of termination, as either continue, soft_stop or hard_stop
       * @return               Indication of success or otherwise
       */
      virtual EReturn terminate(bool & end, double& err);
      
      /**
			 * \brief Setter for error threshold
			 * @param thr[in]   Threshold value
			 * @return        SUCCESS always
			 */
			EReturn setThreshold(const double & thr);

			/**
			 * \brief Getter for error threshold
			 * @param thr[in]   Threshold value
			 * @return        SUCCESS always
			 */
			EReturn getThreshold(double & thr);

    protected:
      /**
       * \brief Derived-Initialisation
       * @param handle XML handle for any derived parameters
       * @return       Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
      
      /// \brief Threshold on squared error.
      double threshold_;
    private:
      Eigen::VectorXd y_;
      int dim_;
      
  };

  typedef exotica::Factory<std::string, exotica::TaskTerminationCriterion> TerminationCriterionCreator; //!< Convenience name for the EndCriterion Singleton Factory
  typedef boost::shared_ptr<exotica::TaskTerminationCriterion> TaskTerminationCriterion_ptr;
}
#endif
