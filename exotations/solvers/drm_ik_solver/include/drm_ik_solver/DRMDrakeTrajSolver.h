/*
 * DRMDrakeTrajSolver.h
 *
 *  Created on: 12 Nov 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRM_IK_SOLVER_DRMDRAKETRAJSOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRM_IK_SOLVER_DRMDRAKETRAJSOLVER_H_

#include <drm_ik_solver/DRMDrakeIKSolver.h>
#include <dynamic_reachability_map/DRMTrajAction.h>

using namespace dynamic_reachability_map;

namespace exotica
{
  class DRMDrakeTrajsolver: public DRMDrakeIKsolver
  {
    public:
      DRMDrakeTrajsolver();
      virtual ~DRMDrakeTrajsolver();
      virtual EReturn Solve(Eigen::VectorXdRefConst q0,
          Eigen::MatrixXd & solution);
      EReturn setGoalState(const Eigen::VectorXd &goal);
      EReturn specifyProblem(PlanningProblem_ptr pointer);
      Eigen::VectorXd qT_;
    protected:
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
    private:
      EReturn IKFilter(Eigen::MatrixXd &solution);
      std::vector<RigidBodyConstraint*> constraints_;
      PostureConstraint* other_cspace_;
      std::vector<std::string> other_joints_;
      std::vector<int> other_joints_idx_;
      actionlib::SimpleActionClient<dynamic_reachability_map::DRMTrajAction> traj_client_;
  };
  typedef boost::shared_ptr<exotica::DRMDrakeTrajsolver> DRMDrakeTrajsolver_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRM_IK_SOLVER_DRMDRAKETRAJSOLVER_H_ */
