#include "kinematic_maps/Distance.h"

REGISTER_TASKMAP_TYPE("Distance", exotica::Distance);

exotica::Distance::Distance()
{
	//!< Empty constructor
}

exotica::EReturn exotica::Distance::update(const Eigen::VectorXd & x, const int t)
{
	//!< Prepare
	invalidate();
	LOCK(scene_lock_);

	//!< Check
	if (scene_ == nullptr)
	{
		INDICATE_FAILURE
		;
		return MMB_NIN;
	}

	//!< Temporaries
	bool success = true;
	EReturn tmp_rtn = FAILURE;

	success = scene_->getForwardMap(object_name_, tmp_phi_);

	if (!success)
	{
		INDICATE_FAILURE
		;
		return FAILURE;
	}
	else
	{
		success = scene_->getJacobian(object_name_, tmp_jac_);
	}
	if (!success)
	{
		INDICATE_FAILURE
		;
		return FAILURE;
	}

	ret_jac_.setZero();
	for (int i = 0; i < scene_->getMapSize(object_name_) / 2; i++)
	{
		ret_phi_(i) = sqrt((tmp_phi_(i * 2 * 3) - tmp_phi_(i * 2 * 3 + 3))
				* (tmp_phi_(i * 2 * 3) - tmp_phi_(i * 2 * 3 + 3))
				+ (tmp_phi_(i * 2 * 3 + 1) - tmp_phi_(i * 2 * 3 + 4))
						* (tmp_phi_(i * 2 * 3 + 1) - tmp_phi_(i * 2 * 3 + 4))
				+ (tmp_phi_(i * 2 * 3 + 2) - tmp_phi_(i * 2 * 3 + 5))
						* (tmp_phi_(i * 2 * 3 + 2) - tmp_phi_(i * 2 * 3 + 5)));

		if (ret_phi_(i) > 1e-50)
		{
			for (int j = 0; j < scene_->getNumJoints(); j++)
			{
				ret_jac_(i, j) = ((tmp_phi_(i * 2 * 3) - tmp_phi_(i * 2 * 3 + 3))
						* (tmp_jac_(i * 2 * 3, j) - tmp_jac_(i * 2 * 3 + 3, j))
						+ (tmp_phi_(i * 2 * 3 + 1) - tmp_phi_(i * 2 * 3 + 4))
								* (tmp_jac_(i * 2 * 3 + 1, j) - tmp_jac_(i * 2 * 3 + 4, j))
						+ (tmp_phi_(i * 2 * 3 + 2) - tmp_phi_(i * 2 * 3 + 5))
								* (tmp_jac_(i * 2 * 3 + 2, j) - tmp_jac_(i * 2 * 3 + 5, j)))
						/ ret_phi_(i);
			}
		}
	}
	{
		tmp_rtn = setPhi(ret_phi_, t);
	}
	if (!success)
	{
		INDICATE_FAILURE
		;
		return FAILURE;
	}
	if (ok(tmp_rtn))
	{
		tmp_rtn = setJacobian(ret_jac_, t);
	}

	return tmp_rtn;
}

exotica::EReturn exotica::Distance::initDerived(tinyxml2::XMLHandle & handle)
{
	if (scene_->getMapSize(object_name_) % 2 != 0)
	{
		ERROR("Kinematic scene must have even number of end-effectors!");
		return FAILURE;
	}
	else
	{
		tmp_phi_.resize(scene_->getMapSize(object_name_) * 3);
		tmp_jac_.resize(scene_->getMapSize(object_name_) * 3, scene_->getNumJoints());
		ret_phi_.resize(scene_->getMapSize(object_name_) / 2);
		ret_jac_.resize(scene_->getMapSize(object_name_) / 2, scene_->getNumJoints());
		return SUCCESS;
	}
}

exotica::EReturn exotica::Distance::taskSpaceDim(int & task_dim)
{
	if (!scene_)
	{
		task_dim = -1;
		ERROR("Kinematic scene has not been initialized!");
		return exotica::MMB_NIN;
	}
	else
	{
		task_dim = scene_->getMapSize(object_name_) / 2;
	}
	return exotica::SUCCESS;
}
