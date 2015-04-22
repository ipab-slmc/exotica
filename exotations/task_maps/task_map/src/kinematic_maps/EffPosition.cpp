#include "kinematic_maps/EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);
REGISTER_FOR_XML_TEST("EffPosition", "EffPosition.xml");

exotica::EffPosition::EffPosition()
{
	//!< Empty constructor
}

exotica::EReturn exotica::EffPosition::update(const Eigen::VectorXd & x, const int t)
{
	//!< Prepare
	invalidate();
	LOCK(scene_lock_);

	//!< Check
	if (scene_ == nullptr)
	{
        INDICATE_FAILURE;
		return MMB_NIN;
	}

    if (ok(scene_->getForwardMap(object_name_, phi_tmp)))
	{
        if(ok(scene_->getJacobian(object_name_, jac_tmp)))
        {
            if(ok(setPhi(phi_tmp, t)) && ok(setJacobian(jac_tmp, t)))
            {
                return SUCCESS;
            }
            else
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
    }
    else
    {
        INDICATE_FAILURE;
		return FAILURE;
	}


}

exotica::EReturn exotica::EffPosition::initDerived(tinyxml2::XMLHandle & handle)
{
    phi_tmp.resize(scene_->getMapSize(object_name_) * 3);
    jac_tmp.resize(scene_->getMapSize(object_name_) * 3, scene_->getNumJoints());
	return SUCCESS;
}

exotica::EReturn exotica::EffPosition::taskSpaceDim(int & task_dim)
{
	if (!scene_)
	{
		task_dim = -1;
		ERROR("Kinematic scene has not been initialized!");
		return exotica::MMB_NIN;
	}
	else
	{
		task_dim = scene_->getMapSize(object_name_) * 3;
	}
	return exotica::SUCCESS;
}
