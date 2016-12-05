/*
 *      Author: Yiming Yang
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

#include "JointLimit.h"

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

REGISTER_TASKMAP_TYPE("JointLimit", exotica::JointLimit);
namespace exotica
{
  JointLimit::JointLimit()
      : initialised_(false)
  {
    //TODO
  }
  JointLimit::~JointLimit()
  {
    //TODO
  }

  void JointLimit::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLElement* xmltmp;
    double percent = 0.1;
    XML_CHECK("SafePercentage");
    getDouble(*xmltmp, percent);

    std::vector<std::string> jnts;
    scene_->getJointNames(jnts);
    int size = jnts.size();
    low_limits_.resize(size);
    high_limits_.resize(size);

    robot_model::RobotModelConstPtr model;
    if (server_->hasParam("RobotDescription")) {
      EParam<std_msgs::String> robot_description_param;
      server_->getParam("RobotDescription", robot_description_param);
      ROS_INFO_STREAM("Loading joint limits for robot_description at " << robot_description_param->data);
      model = server_->getModel(robot_description_param->data);
    } else {
      model = server_->getModel("robot_description");
    }

    for (int i = 0; i < jnts.size(); i++)
    {
      low_limits_(i) =
          model->getJointModel(jnts[i])->getVariableBounds()[0].min_position_;
      high_limits_(i) =
          model->getJointModel(jnts[i])->getVariableBounds()[0].max_position_;
    }
    tau_.resize(size);
    center_.resize(size);
    for (int i = 0; i < size; i++)
    {
      center_(i) = (low_limits_(i) + high_limits_(i)) / 2;
      tau_(i) = percent * (high_limits_(i) - low_limits_(i)) / 2;
    }
    initialised_ = true;
  }

  void JointLimit::taskSpaceDim(int & task_dim)
  {
    if (!initialised_) throw_named("Not initialized!");;
    task_dim = tau_.rows();
  }

  void JointLimit::update(Eigen::VectorXdRefConst x, const int t)
  {
    if (!isRegistered(t))
    {
      throw_named("Not fully initialized!");
    }
    if (!initialised_) throw_named("Not initialized!");;
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
  }
}

