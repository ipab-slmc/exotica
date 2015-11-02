/*
 * DrakeIKProblem.h
 *
 *  Created on: 7 Oct 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_DRAKE_IK_SOLVER_INCLUDE_DRAKEIKPROBLEM_H_
#define EXOTICA_EXOTATIONS_SOLVERS_DRAKE_IK_SOLVER_INCLUDE_DRAKEIKPROBLEM_H_

#include <drake/RigidBodyIK.h>
#include <drake/RigidBodyManipulator.h>
#include <drake/RigidBodyConstraint.h>
#include <drake/IKoptions.h>
#include <exotica/EXOTica.hpp>
#include <exotica/PlanningProblem.h>

namespace exotica
{
  struct Magic
  {
      Magic()
          : r_hand(0), l_hand(0), r_foot(0), l_foot(0)
      {

      }
      int r_hand;
      int l_hand;
      int r_foot;
      int l_foot;
      Eigen::Matrix3Xd l_foot_pts;
      Eigen::Matrix3Xd r_foot_pts;

  };
  class DrakeIKProblem: public PlanningProblem
  {
      friend class DRMDrakeIKProblem;
    public:
      DrakeIKProblem();
      virtual ~DrakeIKProblem();
      virtual EReturn reinitialise(rapidjson::Document& document,
          boost::shared_ptr<PlanningProblem> problem);
      RigidBodyManipulator* getDrakeModel();
      std::vector<RigidBodyConstraint*> constraints_;
    protected:
      /**
       * \brief Derived Initialiser (from XML): PURE VIRTUAL
       * @param handle The handle to the XML-element describing the Problem Definition
       * @return Indication of success/failure
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

    private:
      EReturn buildQuasiStaticConstraint(const rapidjson::Value& obj,
          QuasiStaticConstraint* &ptr);
      EReturn buildPostureConstraint(const rapidjson::Value& obj,
          PostureConstraint* &ptr);
      EReturn buildWorldPositionConstraint(const rapidjson::Value& obj,
          WorldPositionConstraint* &ptr);
      EReturn buildWorldQuatConstraint(const rapidjson::Value& obj,
          WorldQuatConstraint* &ptr);
      EReturn buildFixedLinkFromRobotPoseConstraint(const rapidjson::Value& obj,
          WorldPositionConstraint* &pos_ptr, WorldQuatConstraint* &quat_ptr);
      EReturn getBounds(const rapidjson::Value& obj, Eigen::Vector2d &lb,
          Eigen::Vector2d &ub);
      RigidBodyManipulator* model_;
      std::map<std::string, int> joints_map_;
      Magic magic_;
  };

  typedef boost::shared_ptr<DrakeIKProblem> DrakeIKProblem_ptr;
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_DRAKE_IK_SOLVER_INCLUDE_DRAKEIKPROBLEM_H_ */
