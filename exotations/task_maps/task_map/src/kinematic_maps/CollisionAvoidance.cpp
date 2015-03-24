/*
 * CollisionAvoidance.cpp
 *
 *  Created on: 16 Mar 2015
 *      Author: yiming
 */
#include "kinematic_maps/CollisionAvoidance.h"

REGISTER_TASKMAP_TYPE("CollisionAvoidance", exotica::CollisionAvoidance);

void eigen2Point(const Eigen::Vector3d & eigen, geometry_msgs::Point & point)
{
	point.x = eigen(0);
	point.y = eigen(1);
	point.z = eigen(2);
}
namespace exotica
{
	CollisionAvoidance::CollisionAvoidance()
	{
#ifdef C_DEBUG
		ROS_ERROR("Running collision avoidance taskmap in debug mode");
		state_pub_ = server_->advertise<moveit_msgs::DisplayRobotState>("disp_state", 100);
		close_pub_ = server_->advertise<visualization_msgs::Marker>("close_marker", 100);
		close_.type = visualization_msgs::Marker::LINE_LIST;
		close_.scale.x = 0.004;
		close_.color.g = 1;
		close_.color.a = 1;
#endif
	}

	CollisionAvoidance::~CollisionAvoidance()
	{

	}

	EReturn CollisionAvoidance::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("SafetyRange");
		server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, safe_range_);
		std::cout << "Collision avoidance threshold " << safe_range_->data << " m." << std::endl;
		if (!ok(scene_->getEndEffectors(object_name_, effs_)))
			return MMB_NIN;
		init_offsets_.resize(effs_.size());
		for (int i = 0; i < effs_.size(); i++)
			init_offsets_[i] = KDL::Frame::Identity();
		kin_sol_ = scene_->getSolver();
		kinematica::SolutionForm_t sol;
		sol.end_effector_segs = effs_;
		sol.end_effector_offs = std::vector<KDL::Frame>(effs_.size());
		kin_sol_.updateEndEffectors(sol);
#ifdef C_DEBUG
		close_.header.frame_id = "/" + kin_sol_.getRootName();
		close_pub_ = server_->advertise<visualization_msgs::Marker>("close_marker", 10);
#endif
		return FAILURE;
	}

	EReturn CollisionAvoidance::taskSpaceDim(int & task_dim)
	{
		task_dim = 1;
		return SUCCESS;
	}
	EReturn CollisionAvoidance::update(const Eigen::VectorXd & x, const int t)
	{
		invalidate();
		int M = scene_->getMapSize(object_name_), N = x.rows();
		if (M != effs_.size())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		Eigen::VectorXd eff_phi(3 * M);
		Eigen::MatrixXd eff_jac(3 * M, N);
		if (!ok(scene_->getForwardMap(object_name_, eff_phi))
				|| !ok(scene_->getJacobian(object_name_, eff_jac)))
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		Eigen::VectorXd phi = Eigen::VectorXd::Zero(1);
		std::vector<double> dists(M);
		KDL::Frame tip_offset, cp_offset, eff_offset;
		std::vector<Eigen::Vector3d> norms(M);
		double cost = 0;
#ifdef C_DEBUG
		close_.points.clear();
#endif
		for (int i = 0; i < M; i++)
		{
			Eigen::Vector3d tmp1, tmp2;
			scene_->getCollisionScene()->getRobotDistance(effs_[i], false, dists[i], tmp1, tmp2, norms[i]);
			std::cout << "Dist = " << dists[i] << " Phi=" << phi(0) << std::endl;
			//	Compute Phi
			if (dists[i] < 0)
			{
				std::cerr << "Robot link " << effs_[i] << " is in collision" << std::endl;
				cost = 1;
			}
			else if (dists[i] > safe_range_->data)
				cost = 0;
			else
				cost = (1.0 - dists[i] / safe_range_->data);
			phi(0) += cost * cost;

			//	Modify end-effectors

			tip_offset = KDL::Frame(KDL::Vector(eff_phi(3 * i), eff_phi(3 * i + 1), eff_phi(3 * i
					+ 2)));
			cp_offset = KDL::Frame(KDL::Vector(tmp1(0), tmp1(1), tmp1(2)));
			eff_offset = tip_offset.Inverse() * cp_offset;
			kin_sol_.modifyEndEffector(effs_[i], eff_offset);
#ifdef C_DEBUG
			geometry_msgs::Point p1, p2;
			eigen2Point(tmp1, p1);
			eigen2Point(tmp2, p2);
			close_.points.push_back(p1);
			close_.points.push_back(p2);
#endif
		}

#ifdef C_DEBUG
		close_pub_.publish(close_);
		moveit_msgs::DisplayRobotState msg;
		robot_state::robotStateToRobotStateMsg(scene_->getCollisionScene()->getCurrentState(), msg.state);
		state_pub_.publish(msg);
		ros::spinOnce();
#endif

		if (!kin_sol_.updateConfiguration(x) || !kin_sol_.generateForwardMap()
				|| !kin_sol_.generateJacobian(eff_jac))
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(1, N);
		for (int i = 0; i < M; i++)
		{
			if (dists[i] <= safe_range_->data)
				jac += ((2.0 * dists[i]) / safe_range_->data)
						* (norms[i].transpose() * eff_jac.block(3 * i, 0, 3, N));
		}
		if (!ok(setPhi(phi, t)) || !ok(setJacobian(jac, t)))
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		//	Reset the initial end-effectors

		ROS_ERROR_STREAM("Jac="<<jac);
		return SUCCESS;
	}
}

