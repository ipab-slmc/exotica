/*
 *      Author: Vladimir Ivan
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#include "kinematic_maps/EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);
REGISTER_FOR_XML_TEST("EffPosition", "EffPosition.xml");

namespace exotica
{
  EffPosition::EffPosition()
  {
    //!< Empty constructor
  }

  EReturn EffPosition::initialise(const rapidjson::Value& a)
  {
    std::vector<std::string> tmp_eff(1);
    if (ok(getJSON(a["linkName"], tmp_eff[0])))
    {
      Eigen::VectorXd rel;
      if (ok(getJSON(a["pointInLink"], rel)) && rel.rows() == 3)
      {
        std::vector<KDL::Frame> tmp_offset(1);
        tmp_offset[0] = KDL::Frame::Identity();
        tmp_offset[0].p[0] = rel(0);
        tmp_offset[0].p[1] = rel(1);
        tmp_offset[0].p[2] = rel(2);
        return scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
      }
      else
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }

  }

  EReturn EffPosition::update(Eigen::VectorXdRefConst x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    PHI = EFFPHI;
    if (updateJacobian_)
    {
      JAC = EFFJAC;
    }
    return SUCCESS;
  }

  EReturn EffPosition::initDerived(tinyxml2::XMLHandle & handle)
  {
    return SUCCESS;
  }

  EReturn EffPosition::initialiseManual(std::string name, Server_ptr & server,
      const Scene_map & scene_ptr, boost::shared_ptr<PlanningProblem> prob,
      std::vector<std::pair<std::string, std::string> >& params)
  {
    EReturn ret = TaskMap::initialiseManual(name, server, scene_ptr, prob,
        params);
    initDebug("/world");
    return ret;
  }

  void EffPosition::initDebug(std::string ref)
  {
    eff_mark_.scale.x = eff_mark_.scale.y = eff_mark_.scale.z = 0.1;
    eff_mark_.color.a = eff_mark_.color.b = 1;
    eff_mark_.type = visualization_msgs::Marker::SPHERE_LIST;
    eff_mark_.header.frame_id = ref;
    eff_mark_.ns = getObjectName();
    eff_mark_.points.resize(scene_->getMapSize(object_name_));
    eff_mark_pub_ = server_->advertise<visualization_msgs::Marker>(
        "EffPosition", 1, true);
    HIGHLIGHT(
        "EffPosition is published on ROS topic "<<eff_mark_pub_.getTopic()<<", in reference frame "<<ref);
  }

  void EffPosition::debug()
  {
    if(eff_mark_.points.size()==0)
      initDebug("/world");
    const int eff_size = scene_->getMapSize(object_name_);
    Eigen::Map<Eigen::MatrixXd> eff_mat(EFFPHI.data(), 3, eff_size);
    for (int i = 0; i < eff_size; i++)
    {
      eff_mark_.points[i].x = eff_mat(0, i);
      eff_mark_.points[i].y = eff_mat(1, i);
      eff_mark_.points[i].z = eff_mat(2, i);
    }

    eff_mark_.header.stamp = ros::Time::now();
    eff_mark_pub_.publish(eff_mark_);
  }

  EReturn EffPosition::taskSpaceDim(int & task_dim)
  {
    if (!scene_)
    {
      task_dim = -1;
      ERROR("Kinematic scene has not been initialized!");
      return MMB_NIN;
    }
    else
    {
      task_dim = scene_->getMapSize(object_name_) * 3;
    }
    return SUCCESS;
  }
}
