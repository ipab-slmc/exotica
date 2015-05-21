/*
 * JointLimit.cpp
 *
 *  Created on: 22 Jul 2014
 *      Author: yiming
 */

#include "kinematic_maps/JointLimit.h"
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
REGISTER_TASKMAP_TYPE("JointLimit", exotica::JointLimit);
namespace exotica
{
	JointLimit::JointLimit() :
			initialised_(false)
	{
		//TODO
	}
	JointLimit::~JointLimit()
	{
		//TODO
	}

	EReturn JointLimit::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLElement* xmltmp;
		double percent = 0.1;
		XML_CHECK("SafePercentage");
		XML_OK(getDouble(*xmltmp, percent));

		std::vector<std::string> jnts;
		scene_->getJointNames(jnts);
		int size = jnts.size();
		low_limits_.resize(size);
		high_limits_.resize(size);
		robot_model::RobotModelConstPtr model = server_->getModel("robot_description");
		for (int i = 0; i < jnts.size(); i++)
		{
			low_limits_(i) = model->getJointModel(jnts[i])->getVariableBounds()[0].min_position_;
			high_limits_(i) = model->getJointModel(jnts[i])->getVariableBounds()[0].max_position_;
		}
		tau_.resize(size);
		center_.resize(size);
		for (int i = 0; i < size; i++)
		{
			center_(i) = (low_limits_(i) + high_limits_(i)) / 2;
			tau_(i) = percent * (high_limits_(i) - low_limits_(i)) / 2;
		}
		initialised_ = true;
		return SUCCESS;
	}

	EReturn JointLimit::taskSpaceDim(int & task_dim)
	{
		if (!initialised_)
			return MMB_NIN;
		task_dim = tau_.rows();
		return SUCCESS;
	}

	EReturn JointLimit::update(Eigen::VectorXdRefConst x, const int t)
	{
		if (!isRegistered(t))
		{
			INDICATE_FAILURE
			;
			return FAILURE;
		}
		if (!initialised_)
			return MMB_NIN;
		//	Compute Phi and Jac
		PHI.setZero();
		JAC.setZero();
		double d;
		for (int i = 0; i < PHI.rows(); i++)
		{
			if (x(i) < center_(i))
			{
				d = x(i) - low_limits_(i);
				if (d < tau_(i))
				{
					PHI(i) = tau_(i) - d;
					if (updateJacobian_)
						JAC(i, i) = -1;
				}
			}
			else if (x(i) > center_(i))
			{
				d = high_limits_(i) - x(i);
				if (d < tau_(i))
				{
					PHI(i) = tau_(i) - d;
					if (updateJacobian_)
						JAC(i, i) = 1;
				}
			}
		}
		return SUCCESS;
	}
}

