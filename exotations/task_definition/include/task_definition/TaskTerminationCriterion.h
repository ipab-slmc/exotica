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

/**
 * \brief Convenience registrar for the Termination Criterion Type
 */
#define REGISTER_ENDCRITERION_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::Terminator, TYPE, DERIV)

namespace exotica
{
  class TaskTerminationCriterion : public TaskDefinition
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskTerminationCriterion();
      virtual ~TaskTerminationCriterion(){};
      
      /**
       * \brief Initialiser
       * @post         Guaranteed to call the derived method
       * @param handle XML handle to the node describing the termination criterion
       * @return       SUCCESS if everything goes well (including the initDerived)
       *               PAR_ERR if fails to initialise the criterion strength
       *               specific error from the initDerived if applicable
       */
      EReturn initBase(tinyxml2::XMLHandle & handle);
      
      /**
       * \brief Terminate Query: PURE VIRTUAL
       * @pre                  Should check initialisation is ok
       * @param terminate[out] Returns indication of termination, as either continue, soft_stop or hard_stop
       * @return               Indication of success or otherwise
       */
      virtual EReturn terminate(ETerminate & end) = 0;
      
    protected:
      /**
       * \brief Derived-Initialisation
       * @param handle XML handle for any derived parameters
       * @return       Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle) = 0;
      
      /**
       * \brief Controlled access to the type of the termination condition
       *              Should only be called if the termination condition is met to indicate the type of termination
       * @param type  The ETerminate type (soft or hard)
       * @return      SUCCESS if ok, MMB_NIN if the initialisation is not valid
       */
      EReturn getStrength(ETerminate & strength);
      
    private:
      ETerminate    strength_;        //!< The 'stength' of the termination criterion
      boost::mutex  strength_lock_;   //!< Lock for thread-safety
      
      /**
       * \brief Private setter for the termination criterion: no error checking!
       * @param strength ETerminate strength
       */
      void setStrength(const ETerminate & strength);
      
  };

  typedef exotica::Factory<std::string, exotica::TaskTerminationCriterion> TerminationCriterionCreator; //!< Convenience name for the EndCriterion Singleton Factory
}
#endif
