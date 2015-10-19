/*
 * DRMDrakeIKProblem.h
 *
 *  Created on: 13 Oct 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRM_IK_SOLVER_DRMDRAKEIKPROBLEM_H_
#define EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRM_IK_SOLVER_DRMDRAKEIKPROBLEM_H_

#include "drake_ik_solver/DrakeIKProblem.h"

namespace exotica
{
  class DRMDrakeIKProblem: public DrakeIKProblem
  {
    public:
      DRMDrakeIKProblem();
      virtual ~DRMDrakeIKProblem();

      EReturn reinitialise(rapidjson::Document& document,
          boost::shared_ptr<PlanningProblem> problem);
      std::string eff_link_;
      std::string base_link_;
      int base_pos_con_index_;
      KDL::Frame eff_goal_pose;
    protected:
      EReturn initDerived(tinyxml2::XMLHandle & handle);

  };
  typedef boost::shared_ptr<DRMDrakeIKProblem> DRMDrakeIKProblem_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_DRM_IK_SOLVER_INCLUDE_DRM_IK_SOLVER_DRMDRAKEIKPROBLEM_H_ */
