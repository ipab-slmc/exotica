/*
 * JointSpaceSampling.cpp
 *
 *  Created on: 18 Mar 2015
 *      Author: yiming
 */

#include "kinematic_maps/JointSpaceSampling.h"

namespace exotica
{
	JointSpaceSampling::JointSpaceSampling() :
			stateValid_(false)
	{
		//TODO
	}

	JointSpaceSampling::~JointSpaceSampling()
	{
		//TODO
	}

	EReturn JointSpaceSampling::initDerived(tinyxml2::XMLHandle & handle)
	{

		return SUCCESS;
	}

	EReturn JointSpaceSampling::update(const Eigen::VectorXd & x, const int t)
	{
		return SUCCESS;
	}

	EReturn JointSpaceSampling::isStateValid(bool valid)
	{

		return SUCCESS;
	}
}

