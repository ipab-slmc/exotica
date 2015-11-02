#include "kinematic_maps/EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);
REGISTER_FOR_XML_TEST("EffPosition", "EffPosition.xml");

namespace exotica
{
  EffPosition::EffPosition()
  {
    //!< Empty constructor
  }

  EReturn EffPosition::initialise(const rapidjson::Value& a)
  {
    std::vector<std::string> tmp_eff(1);
    if (ok(getJSON(a["linkName"], tmp_eff[0])))
    {
      Eigen::VectorXd rel;
      if (ok(getJSON(a["pointInLink"], rel)) && rel.rows() == 3)
      {
        std::vector<KDL::Frame> tmp_offset(1);
        tmp_offset[0] = KDL::Frame::Identity();
        tmp_offset[0].p[0] = rel(0);
        tmp_offset[0].p[1] = rel(1);
        tmp_offset[0].p[2] = rel(2);
        return scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
      }
      else
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }

  }

  EReturn EffPosition::update(Eigen::VectorXdRefConst x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    PHI = EFFPHI;
    if (updateJacobian_)
    {
      JAC = EFFJAC;
    }
    return SUCCESS;
  }

  EReturn EffPosition::initDerived(tinyxml2::XMLHandle & handle)
  {
    return SUCCESS;
  }

  EReturn EffPosition::taskSpaceDim(int & task_dim)
  {
    if (!scene_)
    {
      task_dim = -1;
      ERROR("Kinematic scene has not been initialized!");
      return MMB_NIN;
    }
    else
    {
      task_dim = scene_->getMapSize(object_name_) * 3;
    }
    return SUCCESS;
  }
}
