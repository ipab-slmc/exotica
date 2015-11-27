/*
 * DRMDrakeIKProblem.cpp
 *
 *  Created on: 14 Oct 2015
 *      Author: yiming
 */

#include "drm_ik_solver/DRMDrakeIKProblem.h"

REGISTER_PROBLEM_TYPE("DRMDrakeIKProblem", exotica::DRMDrakeIKProblem);

namespace exotica
{
  DRMDrakeIKProblem::DRMDrakeIKProblem()
      : DrakeIKProblem::DrakeIKProblem()
  {

  }

  DRMDrakeIKProblem::~DRMDrakeIKProblem()
  {
//    DrakeIKProblem::~DrakeIKProblem();
  }

  EReturn DRMDrakeIKProblem::reinitialise(rapidjson::Document& document,
      boost::shared_ptr<PlanningProblem> problem)
  {
    if (!ok(DrakeIKProblem::reinitialise(document, problem)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    bool pos_ok = false, quat_ok = false;
    KDL::Frame pointInLink, pos, quat;
    if (document.IsArray())
    {
      for (rapidjson::SizeType i = 0; i < document.Size(); i++)
      {
        rapidjson::Value& obj = document[i];
        if (obj.IsObject())
        {
          std::string constraintClass;
          getJSON(obj["class"], constraintClass);
          if (constraintClass.compare("PositionConstraint") == 0)
          {
            std::string link_name;
            getJSON(obj["linkName"], link_name);

            if (link_name.compare(eff_link_) == 0)
            {
              Eigen::VectorXd tmp;
              if (!ok(getJSON(obj["pointInLink"], tmp)))
              {
                INDICATE_FAILURE
                return FAILURE;
              }
              pointInLink.p[0] = tmp(0);
              pointInLink.p[1] = tmp(1);
              pointInLink.p[2] = tmp(2);
              if (!ok(getJSON(obj["referenceFrame"], pos)))
              {
                INDICATE_FAILURE
                return FAILURE;
              }
              pos_ok = true;
            }
          }
          else if (constraintClass.compare("QuatConstraint") == 0)
          {
            std::string link_name;
            if (ok(getJSON(obj["linkName"], link_name)))
            {
              if (link_name.compare(eff_link_) == 0)
              {
                if (!ok(getJSON(obj["quaternion"], quat)))
                {
                  INDICATE_FAILURE
                  return FAILURE;
                }
                quat_ok = true;
              }
            }
          }
        }
        else
        {
          INDICATE_FAILURE
          return FAILURE;
        }
      }
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    if (!pos_ok)
    {
      WARNING("Position constraint not found");
    }
    else
    {
      pos.M = quat.M;
//    pos.p.data[2] -= 1.025;
      eff_goal_pose = pos * (pointInLink.Inverse());
      HIGHLIGHT_NAMED(object_name_,
          "Eff: "<<eff_link_<<" goalpos ("<<eff_goal_pose.p[0]<<" "<<eff_goal_pose.p[1]<<" "<<eff_goal_pose.p[2]<<")");
    }
    return SUCCESS;
  }

  EReturn DRMDrakeIKProblem::initDerived(tinyxml2::XMLHandle & handle)
  {
    if (!ok(DrakeIKProblem::initDerived(handle)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    tinyxml2::XMLHandle eff_handle = handle.FirstChildElement("EffLink");
    if (!eff_handle.ToElement())
    {
      ERROR("EffLink needs to be specified !");
      return FAILURE;
    }
    eff_link_ = eff_handle.ToElement()->GetText();
    if (model_->findLinkId(eff_link_) == -1)
    {
      ERROR("Link "<<eff_link_<<" not found in robot model");
      return FAILURE;
    }
    tinyxml2::XMLHandle base_handle = handle.FirstChildElement("BaseLink");
    if (!base_handle.ToElement())
    {
      ERROR("EffLink needs to be specified !");
      return FAILURE;
    }
    base_link_ = base_handle.ToElement()->GetText();
//    if (model_->findLinkId(base_link_) == -1)
//    {
//      ERROR("Link "<<base_link_<<" not found in robot model");
//      return FAILURE;
//    }
    return SUCCESS;
  }
}

