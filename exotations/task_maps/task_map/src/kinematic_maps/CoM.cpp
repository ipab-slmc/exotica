/*
 * CoM.cpp
 *
 *  Created on: 21 Mar 2014
 *      Author: yimingyang
 */

#include "kinematic_maps/CoM.h"

REGISTER_TASKMAP_TYPE("CoM", exotica::CoM);

exotica::CoM::CoM() :
		nh_("CoMNode")
{
	initialised_ = false;
	com_pub_ = nh_.advertise<visualization_msgs::Marker>("coms_marker", 1);
	COM_pub_ = nh_.advertise<visualization_msgs::Marker>("COM_marker", 1);
	goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
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
		INDICATE_FAILURE
		return MMB_NIN;
	}

	//!< Temporaries
	bool success = true;
	Eigen::VectorXd phi;
	Eigen::MatrixXd jac;

	if (offset_callback_)
		offset_callback_(this, x, t);

	if (!computeForwardMap(phi))
	{
		INDICATE_FAILURE
		return FAILURE;
	}
	if (!computeJacobian(jac))
	{
		INDICATE_FAILURE
		return FAILURE;
	}

	if (setPhi(phi, t) != SUCCESS)
	{
		INDICATE_FAILURE
		return FAILURE;
	}
	if (setJacobian(jac, t) != SUCCESS)
	{
		INDICATE_FAILURE
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
	task_dim = dim_;
	return SUCCESS;
}
bool exotica::CoM::computeForwardMap(Eigen::VectorXd & phi)
{
	if (!initialised_)
	{
		return false;
	}

	int N = mass_.rows(), i;
    Eigen::VectorXd eff_phi(3 * N);
    scene_->getForwardMap(object_name_, eff_phi);
	KDL::Vector com;
	double M = mass_.sum();

	for (i = 0; i < N; i++)
	{
		KDL::Frame tmp_frame(KDL::Vector(eff_phi(3 * i), eff_phi(3 * i + 1), eff_phi(3 * i + 2)));
		com = com + mass_[i] * tmp_frame.p;
		if (debug_->data)
		{
			geometry_msgs::Point tmp;
            tmp_frame=marker_offset_*tmp_frame;
			tmp.x = tmp_frame.p.data[0];
			tmp.y = tmp_frame.p.data[1];
			tmp.z = tmp_frame.p.data[2];
			com_marker_.points[i] = tmp;
		}
	}

	com = com / M;

	phi.resize(dim_);
	for (int i = 0; i < dim_; i++)
		phi(i) = com.data[i];
	if (!enable_z_->data)
		com.data[2] = 0;
	if (debug_->data)
	{
        KDL::Vector tmp_frame=marker_offset_*com;
        COM_marker_.pose.position.x = tmp_frame[0];
        COM_marker_.pose.position.y = tmp_frame[1];
        COM_marker_.pose.position.z = tmp_frame[2];

		COM_marker_.header.stamp = com_marker_.header.stamp = goal_marker_.header.stamp =
				ros::Time::now();
		com_pub_.publish(com_marker_);
		COM_pub_.publish(COM_marker_);
		goal_pub_.publish(goal_marker_);
		ros::spinOnce();
	}
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
		INDICATE_FAILURE
		return false;
	}
	int N = eff_jac.cols(), i, M = eff_jac.rows() / 3;
	if (mass_.size() != M)
	{
		INDICATE_FAILURE
		return false;
	}
	jac = Eigen::MatrixXd::Zero(dim_, N);
	for (i = 0; i < M; i++)
	{
		jac += mass_[i] / mass_.sum() * eff_jac.block(3 * i, 0, dim_, N);
	}
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
	tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("EnableZ");
	server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, enable_z_);

	if (enable_z_->data)
		dim_ = 3;
	else
		dim_ = 2;
	tmp_handle = handle.FirstChildElement("Debug");
	server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, debug_);
	if (debug_->data)
	{
		com_marker_.points.resize(cog_.size());
		com_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
		com_marker_.color.a = .7;
		com_marker_.color.r = 0.5;
		com_marker_.color.g = 0;
		com_marker_.color.b = 0;
		com_marker_.scale.x = com_marker_.scale.y = com_marker_.scale.z = .02;
		com_marker_.action = visualization_msgs::Marker::ADD;

		COM_marker_.type = visualization_msgs::Marker::CYLINDER;
		COM_marker_.color.a = 1;
		COM_marker_.color.r = 1;
		COM_marker_.color.g = 0;
		COM_marker_.color.b = 0;
		COM_marker_.scale.x = COM_marker_.scale.y = .15;
		COM_marker_.scale.z = .02;
		COM_marker_.action = visualization_msgs::Marker::ADD;

		goal_marker_ = COM_marker_;
		goal_marker_.color.r = 0;
		goal_marker_.color.g = 1;
		std::string save_path = ros::package::getPath("wall_stepping").append("/resources/")
				+ "/CoM/com.txt";
		com_file_.open(save_path.c_str());
//	if (!com_file_.is_open())
//	{
//		INDICATE_FAILURE
//		return FAILURE;
//	}
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
    if (!scene_->updateEndEffectors(object_name_, com_offs))
	{
		INDICATE_FAILURE
		;
		return false;
	}
	return true;
}

exotica::EReturn exotica::CoM::setOffsetCallback(
		boost::function<void(CoM*, const Eigen::VectorXd &, int)> offset_callback)
{
	offset_callback_ = offset_callback;
	return SUCCESS;
}

exotica::EReturn exotica::CoM::setOffset(bool left, const KDL::Frame & offset)
{
	if (debug_->data)
	{
		if (offset == KDL::Frame::Identity())
			INDICATE_FAILURE
		if (left)
		{
            com_marker_.header.frame_id = "base_link";
            COM_marker_.header.frame_id = "base_link";
            goal_marker_.header.frame_id = "base_link";
		}
		else
		{
            com_marker_.header.frame_id = "base_link";
            COM_marker_.header.frame_id = "base_link";
            goal_marker_.header.frame_id = "base_link";
        }
        marker_offset_=offset.Inverse();
	}
	return SUCCESS;
}

void exotica::CoM::checkGoal(const Eigen::Vector3d & goal_)
{
	if (debug_->data)
	{
        KDL::Vector goal = marker_offset_*KDL::Vector(goal_(0),goal_(1),goal_(2));
        goal_marker_.pose.position.x = goal[0];
        goal_marker_.pose.position.y = goal[1];
		if (!enable_z_->data)
			goal_marker_.pose.position.z = 0;
		else
            goal_marker_.pose.position.z = goal[2];
	}
}
