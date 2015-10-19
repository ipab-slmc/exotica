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
    ros::Time start = ros::Time::now();
    Eigen::VectorXd q_sol(prob_->getDrakeModel()->num_positions);
    std::vector<std::string> infeasible_constraint;

    inverseKin(prob_->getDrakeModel(), Eigen::VectorXd(q0), Eigen::VectorXd(q0),
        prob_->constraints_.size(), &prob_->constraints_[0], q_sol, info,
        infeasible_constraint, *ik_options_);
//    inverseKin(prob_->getDrakeModel(),
//        has_last_solve_ ? last_solve_ : Eigen::VectorXd(q0),
//        has_last_solve_ ? last_solve_ : Eigen::VectorXd(q0),
//        prob_->constraints_.size(), &prob_->constraints_[0], q_sol, info,
//        infeasible_constraint, *ik_options_);
    solution.resize(1, prob_->getDrakeModel()->num_positions);
    solution.row(0) = q_sol;
    //Statistics shows using last solution will make it even slower.. :(
//    last_solve_ = q_sol;
//    if (!has_last_solve_) has_last_solve_ = true;
    planning_time_ = ros::Duration(ros::Time::now() - start);
    for (int i = 0; i < infeasible_constraint.size(); i++)
      ERROR("Infeasible: "<<infeasible_constraint[i]);
    return SUCCESS;
  }

  bool DrakeIKsolver::isSolvable(const PlanningProblem_ptr & prob)
  {
    return prob->type().compare("exotica::DrakeIKProblem") == 0;
  }

  EReturn DrakeIKsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    has_last_solve_ = false;
    return SUCCESS;
  }

  EReturn DrakeIKsolver::specifyProblem(PlanningProblem_ptr pointer)
  {
    prob_ = boost::static_pointer_cast<DrakeIKProblem>(pointer);
    ik_options_ = new IKoptions(prob_->getDrakeModel());
//    last_solve_.setZero(prob_->getDrakeModel()->num_positions);
    return SUCCESS;
  }
}
