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
	}

	EReturn CollisionAvoidance::setObsFrame(const KDL::Frame & tf)
	{
		std::vector<double> q(4);
		tf.M.GetQuaternion(q[0], q[1], q[2], q[3]);
		fcl::Quaternion3f quat(q[3], q[0], q[1], q[2]);
		fcl::Vec3f vec(tf.p.x(), tf.p.y(), tf.p.z() + 0.05);
		obs_in_base_tf_.setTransform(quat, vec);
		return SUCCESS;
	}

	CollisionAvoidance::~CollisionAvoidance()
	{

	}

	EReturn CollisionAvoidance::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("SafetyRange");
		server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, safe_range_);

		tmp_handle = handle.FirstChildElement("SelfCollision");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, self_);

		tmp_handle = handle.FirstChildElement("IndicateClear");
		server_->registerParam<std_msgs::Bool>(server_->getName(), tmp_handle, isClear_);
		isClear_->data = true;
		tmp_handle = handle.FirstChildElement("PrintWhenInCollision");
		server_->registerParam<std_msgs::Bool>(server_->getName(), tmp_handle, printWhenInCollision_);
		tmp_handle = handle.FirstChildElement("HardConstrain");
		if (!ok(server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, hard_)))
		{
			hard_->data = false;
		}
		if (!ok(scene_->getEndEffectors(object_name_, effs_)))
			return MMB_NIN;
		init_offsets_.assign(effs_.size(), KDL::Frame::Identity());

		kin_sol_ = scene_->getSolver();
		kinematica::SolutionForm_t sol;
		sol.end_effector_segs = effs_;
		sol.end_effector_offs = std::vector<KDL::Frame>(effs_.size());
		kin_sol_.updateEndEffectors(sol);

		tmp_handle = handle.FirstChildElement("UseVelocity");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, use_vel_);
		if (use_vel_->data)
		{
			vels_.setZero(3 * effs_.size());
			old_eff_phi_.setZero(3 * effs_.size());
		}
		tmp_handle = handle.FirstChildElement("VisualDebug");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, visual_debug_);
		if (visual_debug_->data)
		{
			close_pub_ = server_->advertise<visualization_msgs::Marker>(object_name_
					+ "/close_marker", 100);
			robot_centre_pub_ = server_->advertise<visualization_msgs::Marker>(object_name_
					+ "/robot_centre_marker", 100);
			world_centre_pub_ = server_->advertise<visualization_msgs::Marker>(object_name_
					+ "/world_centre_marker", 100);
			close_.type = visualization_msgs::Marker::LINE_LIST;
			close_.scale.x = 0.004;
			close_.color.g = 1;
			close_.color.a = 1;

			robot_centre_.type = visualization_msgs::Marker::SPHERE_LIST;
			robot_centre_.scale.x = close_.scale.y = close_.scale.z = 0.05;
			robot_centre_.color.g = 1;
			robot_centre_.color.a = 1;
			world_centre_ = robot_centre_;
			world_centre_.color.r = 1;
			world_centre_.color.g = 0;

			robot_centre_.header.frame_id = close_.header.frame_id = "/" + kin_sol_.getRootName();
			world_centre_.header.frame_id = close_.header.frame_id = "/" + kin_sol_.getRootName();
			HIGHLIGHT_NAMED(object_name_, "Collision avoidance taskmap running in debug mode, collision info will be published to marker msgs.");
		}
		return SUCCESS;
	}

	EReturn CollisionAvoidance::setPreUpdateCallback(
			boost::function<void(CollisionAvoidance*, Eigen::VectorXdRefConst, int)> pre_update_callback)
	{
		pre_update_callback_ = pre_update_callback;
		return SUCCESS;
	}

	EReturn CollisionAvoidance::taskSpaceDim(int & task_dim)
	{
		task_dim = 1;
		return SUCCESS;
	}

	bool CollisionAvoidance::isClear()
	{
		return isClear_->data;
	}
	EReturn CollisionAvoidance::update(Eigen::VectorXdRefConst x, const int t)
	{
		if (!isRegistered(t) || !getEffReferences())
		{
			INDICATE_FAILURE
			;
			return FAILURE;
		}

		PHI.setZero();
		if (updateJacobian_)
		{
			JAC.setZero();
		}

		if (pre_update_callback_)
			pre_update_callback_(this, x, t);

		static bool first = true;
		if (use_vel_->data && !first)
		{
			vels_ = EFFPHI - old_eff_phi_;
		}
		int M = EFFPHI.rows() / 3;

		std::vector<double> dists(M);
		std::vector<double> costs(M);
		std::vector<Eigen::Vector3d> c1s(M), c2s(M);
		KDL::Frame tip_offset, cp_offset, eff_offset;
		std::vector<Eigen::Vector3d> norms(M);
		if (visual_debug_->data)
		{
			close_.points.clear();
			robot_centre_.points.clear();
			world_centre_.points.clear();
		}

//		for (auto& objvec : scene_->getCollisionScene()->getFCLWorld())
//		{
//			for (boost::shared_ptr<fcl::CollisionObject> obj : objvec.second)
//			{
//				obj->setTransform(obs_in_base_tf_);
//			}
//		}
		isClear_->data = true;
		Eigen::VectorXd discounts = Eigen::VectorXd::Ones(M);
		if (scene_->getCollisionScene()->getFCLWorld().size() == 0)
			return SUCCESS;
		for (int i = 0; i < M; i++)
		{
			Eigen::Vector3d tmp1, tmp2;
			scene_->getCollisionScene()->getRobotDistance(effs_[i], self_->data, dists[i], tmp1, tmp2, norms[i], c1s[i], c2s[i], safe_range_->data);

			//	Compute Phi
			if (dists[i] <= 0)
			{
				isClear_->data = false;
				if (printWhenInCollision_->data)
					WARNING_NAMED(object_name_, "Robot link " << effs_[i] << " is in collision");
				//	In hard constrain mode, collision == FAILURE
				if (hard_->data)
				{
					return FAILURE;
				}
				costs[i] = 1;
			}
			else if (dists[i] > safe_range_->data)
			{
				costs[i] = 0;
			}
			else
			{
				isClear_->data = false;
				if (use_vel_->data && !first)
				{
					Eigen::Vector3d tmpv = vels_.segment(3 * i, 3);
					double ct = tmpv.dot(norms[i]) / (tmpv.norm() * norms[i].norm());
					if (ct != ct)
					{
						;
					}
					else if (ct < 0)
					{
						discounts(i) = 0;
						HIGHLIGHT_NAMED(effs_[i], "Opposite direction");
					}
					else
					{
						discounts(i) = ct;
						HIGHLIGHT_NAMED(effs_[i], "CosTheta "<<ct<<" Discount "<<discounts(i));
					}
				}
				costs[i] = (1.0 - dists[i] / safe_range_->data);
			}
			PHI(0) = PHI(0) + discounts(i) * (costs[i] * costs[i]);

			//	Modify end-effectors
			if (updateJacobian_)
			{
				tip_offset = KDL::Frame(KDL::Vector(EFFPHI(3 * i), EFFPHI(3 * i + 1), EFFPHI(3 * i
						+ 2)));
				cp_offset = KDL::Frame(KDL::Vector(tmp1(0), tmp1(1), tmp1(2)));
				eff_offset = tip_offset.Inverse() * cp_offset;

				if (!kin_sol_.modifyEndEffector(effs_[i], eff_offset))
				{
					INDICATE_FAILURE
					return FAILURE;
				}
				if (visual_debug_->data)
				{
					geometry_msgs::Point p1, p2;
					if (dists[i] > 0)
					{
						eigen2Point(tmp1, p1);
						eigen2Point(tmp2, p2);
					}
					else
					{
						eigen2Point(c1s[i], p1);
						eigen2Point(c2s[i], p2);
					}
					close_.points.push_back(p1);
					close_.points.push_back(p2);
					eigen2Point(c1s[i], p1);
					eigen2Point(c2s[i], p2);
					robot_centre_.points.push_back(p1);

					world_centre_.points.push_back(p2);
				}
			}
		}

		if (updateJacobian_)
		{
			if (visual_debug_->data)
			{
				close_pub_.publish(close_);
				robot_centre_pub_.publish(robot_centre_);
				world_centre_pub_.publish(world_centre_);
				ros::spinOnce();
			}

			if (kin_sol_.getEffSize() > 0)
			{
				effJac.resize(kin_sol_.getEffSize() * 3, kin_sol_.getNumJoints());
				if (!kin_sol_.updateConfiguration(x) || !kin_sol_.generateForwardMap()
						|| !kin_sol_.generateJacobian(effJac))
				{
					INDICATE_FAILURE
					return FAILURE;
				}

				for (int i = 0; i < M; i++)
				{
					if (dists[i] <= 0)
					{
						Eigen::Vector3d tmpnorm = c2s[i] - c1s[i];
						tmpnorm.normalize();
						JAC += ((2.0 * discounts(i) * costs[i]) / safe_range_->data)
								* (tmpnorm.transpose() * effJac.block(3 * i, 0, 3, JAC.cols()));
					}
					else if (dists[i] <= safe_range_->data)
						JAC += ((2.0 * discounts(i) * costs[i]) / safe_range_->data)
								* (norms[i].transpose() * effJac.block(3 * i, 0, 3, JAC.cols()));
				}
			}
		}

		if (use_vel_->data)
		{
			if (first)
				first = false;
			old_eff_phi_ = EFFPHI;
		}
		return SUCCESS;
	}
}

