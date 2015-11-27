/*
 * DRMDrakeIK.h
 *
 *  Created on: 13 Oct 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRMDRAKEIKSOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRMDRAKEIKSOLVER_H_

#include <drake_ik_solver/DrakeIKSolver.h>
#include <drm_ik_solver/DRMDrakeIKProblem.h>
#include <dynamic_reachability_map/DRMAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace dynamic_reachability_map;

namespace exotica
{
  class DRMDrakeIKsolver: public DrakeIKsolver
  {
    public:
      DRMDrakeIKsolver();
      virtual ~DRMDrakeIKsolver();
      virtual EReturn Solve(Eigen::VectorXdRefConst q0,
          Eigen::MatrixXd & solution);
      EReturn specifyProblem(PlanningProblem_ptr pointer);
    protected:
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
      EParam<StringList> drm_joints_;
      std::vector<int> drm_drake_joints_map_;
      std::vector<int> drm_ps_joints_map_;
      DRMDrakeIKProblem_ptr prob_;
      actionlib::SimpleActionClient<dynamic_reachability_map::DRMAction> drm_client_;
  };
  typedef boost::shared_ptr<exotica::DRMDrakeIKsolver> DRMDrakeIKsolver_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRMDRAKEIKSOLVER_H_ */
