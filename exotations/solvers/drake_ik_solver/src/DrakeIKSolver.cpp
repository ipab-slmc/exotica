/*
 * DrakeIKSolver.cpp
 *
 *  Created on: 8 Oct 2015
 *      Author: yiming
 */

#include "drake_ik_solver/DrakeIKSolver.h"

REGISTER_MOTIONSOLVER_TYPE("DrakeIKsolver", exotica::DrakeIKsolver);

namespace exotica
{
  DrakeIKsolver::DrakeIKsolver()
      : ik_options_(NULL)
  {

  }

  DrakeIKsolver::~DrakeIKsolver()
  {
    if (ik_options_) delete ik_options_;
  }

  EReturn DrakeIKsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    Eigen::VectorXd q_sol(prob_->getDrakeModel()->num_positions);
    std::vector<std::string> infeasible_constraint;

    inverseKin(prob_->getDrakeModel(), Eigen::VectorXd(q0),
        last_solve_.isZero() ? Eigen::VectorXd(q0) : last_solve_,
        prob_->constraints_.size(), &prob_->constraints_[0], q_sol, info,
        infeasible_constraint, *ik_options_);
    solution.resize(1, prob_->getDrakeModel()->num_positions);
    solution.row(0) = q_sol;
    last_solve_ = q_sol;
    return SUCCESS;
  }

  bool DrakeIKsolver::isSolvable(const PlanningProblem_ptr & prob)
  {
    return prob->type().compare("exotica::DrakeIKProblem") == 0;
  }

  EReturn DrakeIKsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    return SUCCESS;
  }

  EReturn DrakeIKsolver::specifyProblem(PlanningProblem_ptr pointer)
  {
    INFO_NAMED(object_name_, "Problem specified");
    prob_ = boost::static_pointer_cast<DrakeIKProblem>(pointer);
    ik_options_ = new IKoptions(prob_->getDrakeModel());
    last_solve_.setZero(prob_->getDrakeModel()->num_positions);
    return SUCCESS;
  }
}
