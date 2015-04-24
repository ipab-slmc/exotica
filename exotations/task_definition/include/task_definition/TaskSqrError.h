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
      virtual ~TaskSqrError(){};
      
      /**
       * \brief Setter for a new goal condition
       * @param y_star[in]  The Goal vector (in task-space co-ordinates)
       * @return            Currently returns SUCCESS always
       */
      EReturn setGoal(const Eigen::Ref<const Eigen::VectorXd> & y_star, int t=0);
      
      /**
       * \brief Getter for the goal vector
       * @param y_star[out] The Goal Vector (task-space)
       * @return            SUCCESS if ok 
       *                    @n MMB_NIN if no goal was previously set
       */
      EReturn getGoal(Eigen::Ref<Eigen::VectorXd> y_star, int t=0);
      
      /**
       * \brief Setter for the intra-Task Weight Matrix W
       * @param W[in]   The Weight Matrix..
       * @return        Always SUCCESS
       */
      EReturn setWeight(const Eigen::Ref<const Eigen::MatrixXd> & W, int t=0);
      
      /**
       * \brief Getter for the Weight Matrix W
       * @param W[out]  The Weight Matrix
       * @return        SUCCESS if there is something to return
       *                @n MMB_NIN if the weight matrix is not initialised
       */
      EReturn getWeight(Eigen::Ref<Eigen::MatrixXd> W, int t=0);
      
      /**
       * \brief Setter for the Inter-Task weight scalar (rho)
       * @param rho[in]   The scalar weight
       * @return        SUCCESS always
       */
      EReturn setRho(const double & rho, int t=0);
      
      /**
       * \brief Getter for the Rho scalar
       * @param rho[out]  The scalar weight
       * @return          SUCCESS if ok
       *                  @n MMB_NIN if not initialised
       */
      EReturn getRho(double & rho, int t=0);
      
      /**
       * \brief	Modify the goal. Currently used for DMeshROS
       * @param	index	Goal entry index
       * @param	value	New goal value
       */
      EReturn modifyGoal(const int & index, const double & value, int t=0);

      /**
       * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
       * @param T Number of time steps (this should be set by the planning problem)
       * @return Returns success.
       */
      virtual EReturn setTimeSteps(const int T);

    protected:
      /**
       * \brief Concrete implementation of the initDerived
       * @param handle  The handle to the XML-element describing the ErrorFunction Function
       * @return        Should indicate success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
      
      /**
       * \brief Resets all parameters to invalid state but does not de-allocate memory!
       */
      void invalidate();
      
    private:
      /** The internal storage **/
      std::vector<Eigen::VectorXd>   y_star_;    //!< The goal vector
      std::vector<Eigen::MatrixXd>   W_;         //!< The Weight matrix
      std::vector<double>            rho_;       //!< The scalar inter-task weight
      bool              y_star_ok_; //!< Initialised flag (goal)
      bool              W_ok_;      //!< Initialised flag (weight)
      bool              rho_ok_;    //!< Initialised flag (rho)
      boost::mutex      y_lock_;    //!< Locking mechanism for the goal vector
      boost::mutex      W_lock_;    //!< Locking mechanism for the weight matrix
      boost::mutex      rho_lock_;  //!< Locking mechanism for rho

  };
  typedef boost::shared_ptr<TaskSqrError> TaskSqrError_ptr;
  class TaskVelocitySqrError : public TaskSqrError
  {

  };
}
#endif
