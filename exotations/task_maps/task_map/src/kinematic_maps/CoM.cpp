/*
 * CoM.cpp
 *
 *  Created on: 21 Mar 2014
 *      Author: yimingyang
 */

#include "kinematic_maps/CoM.h"

REGISTER_TASKMAP_TYPE("CoM", exotica::CoM);

exotica::CoM::CoM()
{
	initialised_ = false;
}

exotica::CoM::~CoM()
{
	//TODO
}

exotica::EReturn exotica::CoM::update(const Eigen::VectorXd & x, const int t)
{

	invalidate();
	LOCK(lock_);
	if (!initialised_)
	{
		return MMB_NIN;
	}

	//!< Temporaries
	bool success = true;
	Eigen::Vector3d phi;
	Eigen::MatrixXd jac;
	if (!scene_->update(x, t))
	{
		return FAILURE;
	}
	if (!computeForwardMap(phi))
	{
		return FAILURE;
	}
	if (!computeJacobian(jac))
	{
		return FAILURE;
	}

	if (setPhi(phi, t) != SUCCESS)
	{

		return FAILURE;
	}
	if (setJacobian(jac, t) != SUCCESS)
	{
		return FAILURE;
	}

	return SUCCESS;
}

exotica::EReturn exotica::CoM::taskSpaceDim(int & task_dim)
{
	if (!initialised_)
	{
		INDICATE_FAILURE
		;
		return MMB_NIN;
	}
	task_dim = 3;
	return SUCCESS;
}
bool exotica::CoM::computeForwardMap(Eigen::Vector3d & phi)
{
	if (!initialised_)
	{
		return false;
	}

	uint N = mass_.rows(), i;
	KDL::Frame tmp;
	KDL::Vector com = KDL::Vector::Zero();
	double M = mass_.sum();

	for (i = 0; i < N; i++)
	{
		tmp = base_pose_[i] * KDL::Frame(cog_[i]);
		com = com + mass_[i] * tmp.p;
	}
	phi(0) = com.x() / M;
	phi(1) = com.y() / M;
	phi(2) = com.z() / M;
	return true;
}

bool exotica::CoM::computeJacobian(Eigen::MatrixXd & jac)
{
	if (!initialised_)
	{
		return false;
	}

	Eigen::MatrixXd eff_jac(mass_.size() * 3, scene_->getMapSize(object_name_));
	if (!scene_->getJacobian(object_name_, eff_jac))
	{
		return false;
	}
	uint N = eff_jac.rows() / 3, i, M = eff_jac.cols();
	if (mass_.size() != N)
	{
		return false;
	}
	jac = Eigen::MatrixXd::Zero(3, eff_jac.cols());
	for (i = 0; i < N; i++)
	{
		jac = jac + mass_[i] * eff_jac.block(3 * i, 0, 3, M);
	}
	jac.transpose();
	return true;
}

exotica::EReturn exotica::CoM::initDerived(tinyxml2::XMLHandle & handle)
{
	if (!changeEffToCoM())
	{
		INDICATE_FAILURE
		;
		return FAILURE;
	}
	initialised_ = true;
	return SUCCESS;
}

bool exotica::CoM::changeEffToCoM()
{
	std::vector<std::string> names;
	if (!scene_->getCoMProperties(names, mass_, cog_, tip_pose_, base_pose_))
	{
		INDICATE_FAILURE
		;
		return false;
	}
	std::vector<KDL::Frame> com_offs;
	int N = names.size(), i;
	com_offs.resize(N);
	for (i = 0; i < N; i++)
	{
		com_offs[i] = tip_pose_[i].Inverse() * base_pose_[i] * KDL::Frame(cog_[i]);
	}
	if (!scene_->updateEndEffectors(object_name_,com_offs))
	{
		INDICATE_FAILURE
		;
		return false;
	}
	return true;
}
