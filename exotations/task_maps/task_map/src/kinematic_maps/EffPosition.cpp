#include "kinematic_maps/EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);
REGISTER_FOR_XML_TEST("EffPosition", "EffPosition.xml");

namespace exotica
{
    EffPosition::EffPosition()
    {
        //!< Empty constructor
    }

    EReturn EffPosition::update(Eigen::VectorXdRefConst x, const int t)
    {
        if(!isRegistered(t)||!getEffReferences()) {INDICATE_FAILURE; return FAILURE;}
        PHI=EFFPHI;
        if(updateJacobian_)
        {
            JAC=EFFJAC;
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
