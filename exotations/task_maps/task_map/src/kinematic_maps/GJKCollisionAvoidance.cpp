/*
 * GJKCollisionAvoidance.cpp
 *
 *  Created on: 7 Jul 2015
 *      Author: yiming
 */

#include "kinematic_maps/GJKCollisionAvoidance.h"

REGISTER_TASKMAP_TYPE("GJKCollisionAvoidance", exotica::GJKCollisionAvoidance);

namespace exotica
{
  GJKCollisionAvoidance::GJKCollisionAvoidance()
  {

  }
  GJKCollisionAvoidance::~GJKCollisionAvoidance()
  {

  }
  EReturn GJKCollisionAvoidance::update(Eigen::VectorXdRefConst x, const int t)
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

    if (pre_update_callback_) pre_update_callback_(this, x, t);

    static bool first = true;

    int M = EFFPHI.rows() / 3;

    std::vector<double> dists(M);
    std::vector<double> cd_(M);
    std::vector<double> expx_(M);
    std::vector<Eigen::Vector3d> c1s(M), c2s(M);
    KDL::Frame tip_offset, cp_offset, eff_offset;
    std::vector<Eigen::Vector3d> norms(M);
    if (visual_debug_->data)
    {
      close_.points.clear();
      robot_centre_.points.clear();
      world_centre_.points.clear();
    }

    isClear_->data = true;
    if (scene_->getCollisionScene()->getFCLWorld().size() == 0) return SUCCESS;
    for (int i = 0; i < M; i++)
    {
      Eigen::Vector3d tmp1, tmp2;
      scene_->getCollisionScene()->getRobotDistance(effs_[i], self_->data,
          dists[i], tmp1, tmp2, norms[i], c1s[i], c2s[i], safe_range_->data);
      //	Compute Phi
      if (dists[i] - safe_range_->data > 0)
      {

        cd_[i] = 0;
      }
      else if (dists[i] <= 0)
      {
        isClear_->data = false;
        if (hard_->data)
        {
          if (printWhenInCollision_->data)
            WARNING_NAMED(object_name_,
                "Robot link " << effs_[i] << " is in collision");
          return FAILURE;
        }
      }
      else
      {
        isClear_->data = false;
        cd_[i] = (1.0 / safe_range_->data) * (dists[i] - safe_range_->data);
        PHI(0) += (-cd_[i] * exp(1.0 / cd_[i]));
      }

      //	Modify end-effectors
      if (updateJacobian_)
      {
        tip_offset = KDL::Frame(
            KDL::Vector(EFFPHI(3 * i), EFFPHI(3 * i + 1), EFFPHI(3 * i + 2)));
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
        double d_ = 0;
        for (int i = 0; i < M; i++)
        {
          if (dists[i] > 0 && dists[i] - safe_range_->data < 0)
          {
            for (int j = 0; j < x.rows(); j++)
            {
              d_ = (c1s[i] - c2s[i]).dot(
                  Eigen::Vector3d(effJac.block(3 * i, j, 3, 1))) / dists[i];
              JAC(0, j) += (1.0 - cd_[i]) / cd_[i] * exp(1.0 / cd_[i]) * d_;
            }
          }
        }
      }
    }
    return SUCCESS;
  }
}		//	Namespace exotica

