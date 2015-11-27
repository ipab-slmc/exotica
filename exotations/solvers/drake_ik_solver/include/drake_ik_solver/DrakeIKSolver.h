/*
 * DrakeIKSolver.h
 *
 *  Created on: 7 Oct 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_DRAKE_IK_SOLVER_INCLUDE_DRAKEIKSOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_DRAKE_IK_SOLVER_INCLUDE_DRAKEIKSOLVER_H_

#include "drake_ik_solver/DrakeIKProblem.h"

namespace exotica
{
  class DrakeIKsolver: public MotionSolver
  {
      friend class DRMDrakeIKsolver;
    public:
      DrakeIKsolver();
      virtual ~DrakeIKsolver();

      /**
       * \brief Solves the problem. This returns the final state
       * @param q0      Start state.
       * @param solution  This will be filled with the solution in joint space(Vector).
       * @return  SUCESS if solution has been found, corresponding error code if not.
       */
      virtual EReturn Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution);

      /**
       * \brief Binds the solver to a specific problem which must be pre-initalised
       * @param pointer Shared pointer to the motion planning problem
       * @return  Successful if the problem is a valid AICOProblem
       */
      virtual EReturn specifyProblem(PlanningProblem_ptr pointer);

      /*
       * \brief Check if a problem is solvable by this solver (Pure Virtual)
       * @param prob    Planning problem
       * @return  True if solvable, false otherwise
       */
      virtual bool isSolvable(const PlanningProblem_ptr & prob);
      int info;
      ros::Duration planning_time_;
    protected:
      /**
       * \brief Derived-elements initialiser: Pure Virtual
       * @param handle  XMLHandle to the Solver element
       * @return  Should indicate success or otherwise
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
      IKoptions* ik_options_;
    private:
      DrakeIKProblem_ptr prob_;
      Eigen::VectorXd last_solve_;
      bool has_last_solve_;
  };
  typedef boost::shared_ptr<exotica::DrakeIKsolver> DrakeIKsolver_ptr;

}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_DRAKE_IK_SOLVER_INCLUDE_DRAKEIKSOLVER_H_ */
