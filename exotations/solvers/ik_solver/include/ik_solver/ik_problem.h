/*
 * ik_problem.h
 *
 *  Created on: 15 Jul 2014
 *      Author: yiming
 */

#ifndef IK_PROBLEM_H_
#define IK_PROBLEM_H_
#include <exotica/PlanningProblem.h>
#include "task_definition/TaskSqrError.h"
#include "kinematic_maps/Distance.h"
#include "kinematic_maps/Orientation.h"
namespace exotica
{
  /**
   * IK problem implementation
   */
  class IKProblem: public PlanningProblem
  {
    public:
      IKProblem();
      virtual ~IKProblem();

      /**
       * \brief	Get configuration weight
       * @return	configuration weight
       */
      Eigen::MatrixXd getW();

      int getT();

      /**
       * \brief	Get tolerance
       * @return	tolerance
       */
      double getTau();
      void setTau(double tau);

      virtual EReturn reinitialise(rapidjson::Document& document,
          boost::shared_ptr<PlanningProblem> problem);
    protected:
      /**
       * \brief Derived Initialiser (from XML): PURE VIRTUAL
       * @param handle The handle to the XML-element describing the Problem Definition
       * @return Indication of success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

      Eigen::MatrixXd config_w_;	//Configuration weight
      double tau_;	// Tolerance
      int T_;

  };
  typedef boost::shared_ptr<exotica::IKProblem> IKProblem_ptr;
}

#endif /* IK_PROBLEM_H_ */
