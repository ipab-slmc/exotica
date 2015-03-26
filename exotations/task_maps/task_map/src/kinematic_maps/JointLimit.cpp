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

		robot_model::RobotModelConstPtr model = scene_->getPlanningScene()->getRobotModel();

		std::vector<std::string> segs, jnts;
		scene_->getSolver().getControlledSegmentsAndJoints(segs, jnts);
		low_limits_.resize(jnts.size());
		high_limits_.resize(jnts.size());
		for (int i = 0; i < jnts.size(); i++)
		{
			low_limits_[i] = model->getVariableBounds(jnts[i]).min_position_;
			high_limits_[i] = model->getVariableBounds(jnts[i]).max_position_;
		}
		if (low_limits_.rows() != high_limits_.rows())
		{
			ERROR("Joint limits Wrong size");
			return FAILURE;
		}
//		ROS_INFO_STREAM("Joint limit task [Lower limits]: "<<low_limits_.transpose());
//		ROS_INFO_STREAM("Joint limit task [Upper limits]: "<<high_limits_.transpose());
		int size = low_limits_.rows();
		double percent = 0.1;
		XML_CHECK("SafePercentage");
		XML_OK(getDouble(*xmltmp, percent));

		tau_.resize(size);
		center_.resize(size);
		for (int i = 0; i < size; i++)
		{
			center_(i) = (low_limits_(i) + high_limits_(i)) / 2;
			tau_(i) = percent * (high_limits_(i) - low_limits_(i)) / 2;
		}

//		ROS_INFO_STREAM("Joint limit task [Safe limits ("<<percent<<")]: "<<tau_.transpose());
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

	EReturn JointLimit::update(const Eigen::VectorXd & x, const int t)
	{
		if (!initialised_)
			return MMB_NIN;
		int size = x.rows();
		if (size != tau_.rows())
			return FAILURE;
		//	Compute Phi and Jac
		Eigen::VectorXd phi = Eigen::VectorXd::Zero(size);
		Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(size, size);
		double d;
		for (int i = 0; i < size; i++)
		{
			if (x(i) < center_(i))
			{
				d = x(i) - low_limits_(i);
				if (d < tau_(i))
				{
					phi(i) = tau_(i) - d;
					jac(i, i) = -1;
				}
			}
			else if (x(i) > center_(i))
			{
				d = high_limits_(i) - x(i);
				if (d < tau_(i))
				{
					phi(i) = tau_(i) - d;
					jac(i, i) = 1;
				}
			}
		}
		setPhi(phi, t);
		setJacobian(jac, t);
		return SUCCESS;
	}
}

