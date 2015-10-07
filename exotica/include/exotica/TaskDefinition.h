/***********************************************************************\
|    This is the base class for all forms of cost-function type         |
 |  implementations. It currently provides no functionality apart from   |
 |  allowing polymorphic behaviour between the different cost            |
 |  components. THREAD-SAFE                                              |
 |                                                                       |
 |           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
 |                    Last Edited: 18 - March - 2014                     |
 \***********************************************************************/

#ifndef EXOTICA_TASK_DEFINITION_H
#define EXOTICA_TASK_DEFINITION_H

#include "exotica/Object.h"       //! The EXOTica base class
#include "exotica/TaskMap.h"      //! The Task map (since we require a ptr to it)
#include "exotica/Tools.h"

#include <Eigen/Dense>
#include <string>
#include <map>

#define REGISTER_TASKDEFINITION_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::TaskDefinition, TYPE, DERIV)

namespace exotica
{
  class TaskDefinition: public Object
  {
    public:
      /**
       * \brief Default Constructor
       */
      TaskDefinition();
      virtual ~TaskDefinition()
      {
      }
      ;

      /**
       * \brief Base Initialiser
       * @pre             The TaskMaps must be initialised
       * @post            Will call the initDerived() function if everything is successful
       * @param handle    Handle to the XML Element
       * @param map_list  A map from names to TaskMap pointers (for initialising the map)
       * @return          The result of calling the initDerived() function
       */
      EReturn initBase(tinyxml2::XMLHandle & handle,
          const TaskMap_map & map_list);

      /**
       * @brief registerPhi Registers a memory location for the output of phi at time t
       * @param y Reference to memory location to be registered
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerPhi(Eigen::VectorXdRef_ptr y, int t);

      /**
       * @brief registerJacobian egisters a memory location for the output of Jacobian at time t
       * @param J Reference to memory location to be registered
       * @param t Time step
       * @return Indication of success
       */
      EReturn registerJacobian(Eigen::MatrixXdRef_ptr J, int t);

      /**
       * \brief Wrapper for the underlying task dimension getter
       * @param  task_dim Task dimension to be returned
       * @return      Indication of success
       */
      EReturn taskSpaceDim(int & task_dim);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual EReturn setTimeSteps(const int T);

      /**
       * \brief Member function for binding the Task definition to a Task-Map
       * @param task_map  Smart pointer to a task-map
       * @return          SUCCESS always.
       */
      EReturn setTaskMap(const TaskMap_ptr & task_map);

      /**
       * \brief Member function for getting the Task-Map
       * @return          Smart pointer to a task-map
       */
      TaskMap_ptr getTaskMap();

      bool order;

      virtual std::string print(std::string prepend);

    protected:
      /**
       * \brief Initialises members of the derived type: PURE_VIRTUAL
       * @param handle  The handle to the XML-element describing the ErrorFunction Function
       * @return        Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle) = 0;

      boost::shared_ptr<TaskMap> task_map_; //!< Shared pointer to a Task Map from which it gets its inputs
      boost::mutex map_lock_;  //!< Mapping Lock for synchronisation

  };

  typedef Factory<std::string, TaskDefinition> TaskDefinition_fac;
  typedef boost::shared_ptr<TaskDefinition> TaskDefinition_ptr;
  typedef std::map<std::string, TaskDefinition_ptr> TaskDefinition_map;

}
#endif
