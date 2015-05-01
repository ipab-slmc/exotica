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
		nh_ = ros::NodeHandle("CollisionAvoidance");
		close_pub_ = nh_.advertise<visualization_msgs::Marker>("close_marker", 100);
		centre_pub_ = nh_.advertise<visualization_msgs::Marker>("centre_marker", 100);
		close_.type = visualization_msgs::Marker::LINE_LIST;
		close_.scale.x = 0.004;
		close_.color.g = 1;
		close_.color.a = 1;

		centre_.type = visualization_msgs::Marker::SPHERE_LIST;
		centre_.scale.x = close_.scale.y = close_.scale.z = 0.05;
		centre_.color.g = 1;
		centre_.color.a = 1;
#endif
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
		centre_.header.frame_id = close_.header.frame_id = "/" + kin_sol_.getRootName();
#endif
        return SUCCESS;
	}

    EReturn CollisionAvoidance::setPreUpdateCallback(boost::function<void(CollisionAvoidance*, Eigen::VectorXdRefConst, int)> pre_update_callback)
    {
        pre_update_callback_ = pre_update_callback;
    }

	EReturn CollisionAvoidance::taskSpaceDim(int & task_dim)
	{
		task_dim = 1;
		return SUCCESS;
	}
	EReturn CollisionAvoidance::update(Eigen::VectorXdRefConst x, const int t)
	{
        if(!isRegistered(t)||!getEffReferences()) {INDICATE_FAILURE; return FAILURE;}
        if (pre_update_callback_) pre_update_callback_(this, x, t);

        int M = EFFPHI.rows()/3;

        PHI.setZero();
		std::vector<double> dists(M);
		std::vector<double> costs(M);
		std::vector<Eigen::Vector3d> c1s(M), c2s(M);
		KDL::Frame tip_offset, cp_offset, eff_offset;
		std::vector<Eigen::Vector3d> norms(M);
#ifdef C_DEBUG
		close_.points.clear();
		centre_.points.clear();
#endif

        for (auto& objvec : scene_->getCollisionScene()->getFCLWorld())
        {
            for (boost::shared_ptr<fcl::CollisionObject> obj : objvec.second)
            {
                obj->setTransform(obs_in_base_tf_);
            }
        }
		for (int i = 0; i < M; i++)
		{
			Eigen::Vector3d tmp1, tmp2;
			scene_->getCollisionScene()->getRobotDistance(effs_[i], false, dists[i], tmp1, tmp2, norms[i], c1s[i], c2s[i]);
			//	Compute Phi
			if (dists[i] <= 0)
			{
#ifdef C_DEBUG
				ROS_ERROR_STREAM("Robot link " << effs_[i] << " is in collision");
#endif
				costs[i] = 1;
			}
			else if (dists[i] > safe_range_->data)
				costs[i] = 0;
			else
				costs[i] = (1.0 - dists[i] / safe_range_->data);

            PHI(0) = PHI(0) + costs[i] * costs[i];

			//	Modify end-effectors
            if(updateJacobian_)
                {
                tip_offset = KDL::Frame(KDL::Vector(EFFPHI(3 * i), EFFPHI(3 * i + 1), EFFPHI(3 * i + 2)));
                cp_offset = KDL::Frame(KDL::Vector(tmp1(0), tmp1(1), tmp1(2)));
                eff_offset = tip_offset.Inverse() * cp_offset;

                if (!kin_sol_.modifyEndEffector(effs_[i], eff_offset))
                {
                    INDICATE_FAILURE
                    return FAILURE;
                }
    #ifdef C_DEBUG
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
                centre_.points.push_back(p1);
                centre_.points.push_back(p2);
    #endif
            }

    #ifdef C_DEBUG
            close_pub_.publish(close_);
            centre_pub_.publish(centre_);
            ros::spinOnce();
    #endif

            if (!kin_sol_.updateConfiguration(x) || !kin_sol_.generateForwardMap()
                    || !kin_sol_.generateJacobian(EFFJAC))
            {
                INDICATE_FAILURE
                return FAILURE;
            }
            JAC.setZero();
            for (int i = 0; i < M; i++)
            {
                if (dists[i] <= 0)
                {
                    Eigen::Vector3d tmpnorm = c2s[i] - c1s[i];
                    tmpnorm.normalize();
                    JAC += ((2.0 * costs[i]) / safe_range_->data)
                            * (tmpnorm.transpose() * EFFJAC.block(3 * i, 0, 3, JAC.cols()));
                }
                else if (dists[i] <= safe_range_->data)
                    JAC += ((2.0 * costs[i]) / safe_range_->data)
                            * (norms[i].transpose() * EFFJAC.block(3 * i, 0, 3, JAC.cols()));
            }
        }
		return SUCCESS;
	}
}

