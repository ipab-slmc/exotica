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

#include "dmesh_ros.h"
REGISTER_TASKMAP_TYPE("DMeshROS", exotica::DMeshROS);
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

namespace exotica
{
  DMeshROS::DMeshROS()
      : initialised_(false), ir_(0.2)
  {
    //TODO
  }

  DMeshROS::~DMeshROS()
  {
    //TODO
  }

  void DMeshROS::Instantiate(DMeshROSInitializer& init)
  {

  }

  void DMeshROS::update(Eigen::VectorXdRefConst x, const int t)
  {
    q_size_ = x.rows();

    if (initialised_)
    {
        computeLaplace(t);
        if (updateJacobian_)
        {
          computeJacobian(t);
        }
    }
    else
    {
      throw_named("Not initialized!");
    }
  }

  void DMeshROS::taskSpaceDim(int & task_dim)
  {
    LOCK(lock_);
    if (!initialised_)
    {
      throw_named("Not initialized!");
    }
    //task_size_ = gManager_.getGraph()->getCurrentSize();
    task_dim = task_size_;
  }

  void DMeshROS::getGoalLaplace(Eigen::VectorXd & goal, int t)
  {
    LOCK(lock_);
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    if (!initialised_)
    {
      throw_named("Not initialized!");
    }

    updateGraphFromKS(t);

    Eigen::MatrixXd dist;
    if (!gManager_.getGraph()->getGoalDistanceEigen(dist))
    {
      throw_named("Failed to get distances!");
    }
    goal.setZero(task_size_);
    uint j, l, cnt = 0;
    double d, b;
    for (j = 0; j < robot_size_; j++)
    {
      int tmp = robot_size_;
      if (usePose_) tmp = j + 2;
      for (l = tmp; l < size_; l++)
      {
        switch (gManager_.getGraph()->getVertex(l)->getType())
        {
        case VERTEX_TYPE::LINK:
          goal(cnt) = kp_ * dist(j, l);
          break;
        case VERTEX_TYPE::OBSTACLE:
          if (gManager_.getGraph()->getVertex(l)->checkList(links_[j]))
          {
            goal(cnt) = ko_;
          }
          break;
        case VERTEX_TYPE::OBSTACLE_TO_ALL:
          goal(cnt) = ko_;
          break;
        case VERTEX_TYPE::GOAL:
          goal(cnt) = gManager_.getGraph()->getVertex(l)->w_
              * gManager_.getGraph()->getVertex(l)->radius_;
          break;
        default:
          //	All other types will zero for goal laplace
          break;
        }
        cnt++;
      }
    }
//		std::cout << "Dist \n" << dist << std::endl;
//		std::cout << "GOAL: " << goal.transpose() << std::endl;
//		getchar();
  }

  void DMeshROS::getLaplace(Eigen::VectorXd & lap)
  {
    int t = 0;
    lap = PHI;
  }

  void DMeshROS::computeLaplace(int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    LOCK(lock_);
    if (!initialised_)
    {
      throw_named("Not initialized!");
    }

    updateGraphFromKS(t);
    if (!gManager_.getGraph()->getAcutalDistanceEigen(dist_))
    {
      throw_named("Couldn't get actual distances!");
    }
    //Eigen::MatrixXd vel(gManager_.getGraph()->getVelocity());
    PHI.setZero();
    uint j, l, cnt = 0;
    double b = 0, d = 0;
    for (j = 0; j < robot_size_; j++)
    {
      int tmp = robot_size_;
      if (usePose_) tmp = j + 2;
      for (l = tmp; l < size_; l++)
      {
        b = d = 0;
        switch (gManager_.getGraph()->getVertex(l)->getType())
        {
        case VERTEX_TYPE::LINK:
          PHI(cnt) = kp_ * dist_(j, l);
          break;
        case VERTEX_TYPE::OBSTACLE:
          if (gManager_.getGraph()->getVertex(l)->checkList(links_[j]))
          {
            if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
                - gManager_.getGraph()->getVertex(l)->getRadius() < 0.05)
              PHI(cnt) = ko_ * (1 - exp(-wo_ * dist_(j, l)));
            else
              PHI(cnt) = ko_;

          }
          break;
        case VERTEX_TYPE::OBSTACLE_TO_ALL:
          if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
              - gManager_.getGraph()->getVertex(l)->getRadius() < 0.05)
            PHI(cnt) = ko_ * (1 - exp(-wo_ * dist_(j, l)));
          else
            PHI(cnt) = ko_;
          break;
        case VERTEX_TYPE::GOAL:
          PHI(cnt) = gManager_.getGraph()->getVertex(l)->w_ * dist_(j, l);
          break;
        default:
          break;
        }
        cnt++;
      }
    }
  }

  void DMeshROS::computeJacobian(int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    JAC.setZero();
    double d_ = 0;
    uint i, j, l, cnt;
    for (i = 0; i < q_size_; i++)
    {
      cnt = 0;
      for (j = 0; j < robot_size_; j++)
      {
        int tmp = robot_size_;
        if (usePose_) tmp = j + 2;
        for (l = tmp; l < size_; l++)
        {
          if (dist_(j, l) > 0)
          {
            switch (gManager_.getGraph()->getVertex(l)->getType())
            {
            case VERTEX_TYPE::LINK:
              d_ = ((gManager_.getGraph()->getVertex(j)->position_
                  - gManager_.getGraph()->getVertex(l)->position_).dot(
                  Eigen::Vector3d(
                      EFFJAC.block(3 * j, i, 3, 1)
                          - EFFJAC.block(3 * l, i, 3, 1)))) / dist_(j, l);
              JAC(cnt, i) = kp_ * d_;
              break;
            case VERTEX_TYPE::OBSTACLE:
              if (gManager_.getGraph()->getVertex(l)->checkList(
                  links_[j]))
              {
                if (dist_(j, l)
                    - gManager_.getGraph()->getVertex(j)->getRadius()
                    - gManager_.getGraph()->getVertex(l)->getRadius() < 0.5)
                {
                  d_ = ((gManager_.getGraph()->getVertex(j)->position_
                      - gManager_.getGraph()->getVertex(l)->position_).dot(
                      Eigen::Vector3d(EFFJAC.block(3 * j, i, 3, 1))))
                      / dist_(j, l);
                  JAC(cnt, i) = ko_ * wo_ * d_ * exp(-wo_ * dist_(j, l));
                }
              }
              break;
            case VERTEX_TYPE::OBSTACLE_TO_ALL:
              if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
                  - gManager_.getGraph()->getVertex(l)->getRadius() < 0.5)
              {
                d_ = ((gManager_.getGraph()->getVertex(j)->position_
                    - gManager_.getGraph()->getVertex(l)->position_).dot(
                    Eigen::Vector3d(EFFJAC.block(3 * j, i, 3, 1))))
                    / dist_(j, l);
                JAC(cnt, i) = ko_ * wo_ * d_ * exp(-wo_ * dist_(j, l));
              }

              break;
            case VERTEX_TYPE::GOAL:
              d_ = ((gManager_.getGraph()->getVertex(j)->position_
                  - gManager_.getGraph()->getVertex(l)->position_).dot(
                  Eigen::Vector3d(EFFJAC.block(3 * j, i, 3, 1)))) / dist_(j, l);
              JAC(cnt, i) = gManager_.getGraph()->getVertex(l)->w_ * d_;
              break;
            default:
              break;
            }
          }
          cnt++;
        }
      }
    }
  }

  void DMeshROS::updateGraphFromKS(int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    if (!gManager_.getGraph()->updateLinksRef(EFFPHI))
    {
      throw_named("Not initialized!");
    }
  }

  void DMeshROS::updateGraphFromExternal(const std::string & name,
      const Eigen::Vector3d & pose)
  {
    if (!gManager_.getGraph()->updateLink(name, pose))
    {
      throw_named("Can't update the link!");
    }
  }

  void DMeshROS::updateGraphFromTF()
  {
    Eigen::VectorXd tmp = Eigen::VectorXd::Zero(robot_size_ * 3);
    for (int i = 1; i < robot_size_ - 1; i++)
    {
      try
      {
        listener_.lookupTransform("/base", "/" + links_[i],
            ros::Time(0), transform_);
      } catch (tf::TransformException &ex)
      {

        ros::Duration(1.0).sleep();
        throw_named(ex.what());
      }
      tmp(3 * i) = transform_.getOrigin().x();
      tmp(3 * i + 1) = transform_.getOrigin().y();
      tmp(3 * i + 2) = transform_.getOrigin().z();
    }
    tmp(3 * (robot_size_ - 1)) = 0.3;
    tmp(3 * (robot_size_ - 1) + 1) = 5;
    tmp(3 * (robot_size_ - 1) + 2) = 0.2;

    if (!gManager_.getGraph()->updateLinksRef(tmp))
    {
      throw_named("Can't update link references!");
    }
  }

  void DMeshROS::updateExternal(const exotica::MeshVertex & ext)
  {
    if (!gManager_.getGraph()->updateExternal(ext))
    {
      ROS_ERROR_STREAM("Update "<<ext.name<<" failed");
    }
  }

  void DMeshROS::updateExternal(const exotica::MeshVertexArray & ext)
  {
    for (int i = 0; i < ext.vertices.size(); i++)
    {
      updateExternal(ext.vertices[i]);
    }
  }

  void DMeshROS::removeVertex(const std::string & name)
  {
    if (!gManager_.getGraph()->removeVertex(name))
    {
      ROS_ERROR_STREAM("Remove "<<name<<" failed");
    }
  }

  bool DMeshROS::hasActiveObstacle()
  {
    return gManager_.getGraph()->hasActiveObstacle();
  }
}

