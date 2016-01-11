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

#include "kinematic_maps/DMesh.h"
REGISTER_TASKMAP_TYPE("DMesh", exotica::DMesh);
namespace exotica
{
  DMesh::DMesh()
  {

  }

  DMesh::~DMesh()
  {

  }

  EReturn DMesh::initDerived(tinyxml2::XMLHandle & handle)
  {
    EParam<std_msgs::Int64> tmp;
    if (!ok(server_->getParam(server_->getName() + "/DMeshSize", tmp)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    graph_size_ = tmp->data;

    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("DMeshLinks");
    server_->registerParam<exotica::StringList>(ns_, tmp_handle, links_);

    tmp_handle = handle.FirstChildElement("DMeshLinkTypes");
    server_->registerParam<exotica::BoolList>(ns_, tmp_handle, link_types_);

    tmp_handle = handle.FirstChildElement("PoseGain");
    if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, kp_)))
      kp_->data = 0;

    tmp_handle = handle.FirstChildElement("ObstacleGain");
    if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, ko_)))
      ko_->data = 10;

    tmp_handle = handle.FirstChildElement("GoalGain");
    if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, kg_)))
      kg_->data = 100;

    tmp_handle = handle.FirstChildElement("ExpGain");
    if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, kexp_)))
      kexp_->data = 1;

    tmp_handle = handle.FirstChildElement("InteractRange");
    if (!ok(server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, di_)))
      di_->data = 0.5;
    ///	Initialise the mesh
    vertices_.resize(graph_size_);
    vertex_map_.clear();
    current_size_ = links_->strings.size();
    for (int i = 0; i < current_size_; i++)
    {
      vertices_[i].reset(
          new Vertex(links_->strings[i],
              link_types_->data[i] ? Vertex::LINK : Vertex::DUMMY_LINK, i));
      vertex_map_[links_->strings[i]] = i;
      if (kp_->data != 0) for (int j = i + 1; j < current_size_; j++)
        vertices_[i]->addNewRelation(links_->strings[i], i);
    }

    robot_size_ = current_size_;
    ext_size_ = graph_size_ - current_size_;
    task_size_ =
        (kp_->data == 0) ?
            current_size_ * (graph_size_ - current_size_) :
            (graph_size_ * (graph_size_ - 1) / 2 - current_size_);
    dist_.setZero(graph_size_, graph_size_);
    HIGHLIGHT_NAMED("DMesh",
        "Distance Mesh has been initialised: Maximum Graph size="<<graph_size_<<", Robot link size="<<robot_size_<<", Unconnected external object size="<<ext_size_);
    return SUCCESS;
  }

  EReturn DMesh::taskSpaceDim(int & task_dim)
  {
    LOCK(lock_);
    task_dim = task_size_;
    return SUCCESS;
  }

  EReturn DMesh::update(Eigen::VectorXdRefConst x, const int t)
  {
    q_size_ = x.rows();
    if (!isRegistered(t) || !getEffReferences())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (!ok(updateDistances()) || !ok(updatePositions()))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (ok(computeLaplace(t)))
    {
      if (updateJacobian_)
      {
        if (ok(computeJacobian(t)))
        {
          return SUCCESS;
        }
        else
        {
          INDICATE_FAILURE
          return FAILURE;
        }
      }
      else
      {
        return SUCCESS;
      }
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
  }

  EReturn DMesh::getGoalLaplace(Eigen::VectorXd & goal, int t)
  {
    LOCK(lock_);
    if (!isRegistered(t) || !getEffReferences())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (!ok(updateDistances()) || !ok(updatePositions()))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    goal.setZero(task_size_);
    uint j, l, cnt = 0;
    double d, b;
    for (j = 0; j < robot_size_; j++)
    {
      for (l = robot_size_; l < graph_size_; l++)
      {
        switch (vertices_[l]->type_)
        {
        case Vertex::LINK:
          goal(cnt) = kp_->data * dist_(j, l);
          break;
        case Vertex::OBSTACLE_TO_ALL:
          goal(cnt) = ko_->data;
          break;
        default:
          //	All other types will zero for goal laplace
          break;
        }
        cnt++;
      }
    }

    return SUCCESS;
  }
  EReturn DMesh::computeLaplace(int t)
  {
    LOCK(lock_);
    PHI.setZero();
    uint j, l, cnt = 0;
    double b = 0, d = 0;
    for (j = 0; j < robot_size_; j++)
    {
      for (l = robot_size_; l < graph_size_; l++)
      {
        b = d = 0;
        switch (vertices_[l]->type_)
        {
        case Vertex::LINK:
          PHI(cnt) = kp_->data * dist_(j, l);
          break;
        case Vertex::OBSTACLE_TO_ALL:
          PHI(cnt) = ko_->data * (1 - exp(-kexp_->data * dist_(j, l)));
          break;
        case Vertex::GOAL:
          PHI(cnt) = kg_->data * (1 - exp(-kexp_->data * dist_(j, l)));
          break;
        default:
          break;
        }
        cnt++;
      }
    }
    return SUCCESS;
  }

  EReturn DMesh::computeJacobian(int t)
  {
    JAC.setZero();
    double d_ = 0;
    uint i, j, l, cnt;
    for (i = 0; i < q_size_; i++)
    {
      cnt = 0;
      for (j = 0; j < robot_size_; j++)
      {
        for (l = robot_size_; l < graph_size_; l++)
        {
          if (dist_(j, l) > 0)
          {
            switch (vertices_[l]->type_)
            {
            case Vertex::LINK:
              d_ = ((vertices_[j]->position_ - vertices_[l]->position_).dot(
                  Eigen::Vector3d(
                      EFFJAC.block(3 * j, i, 3, 1)
                          - EFFJAC.block(3 * l, i, 3, 1)))) / dist_(j, l);
              JAC(cnt, i) = kp_->data * d_;
              break;
            case Vertex::OBSTACLE_TO_ALL:
              if (dist_(j, l) < 0.5)
              {
                d_ = ((vertices_[j]->position_ - vertices_[l]->position_).dot(
                    Eigen::Vector3d(EFFJAC.block(3 * j, i, 3, 1))))
                    / dist_(j, l);
                JAC(cnt, i) = ko_->data * kexp_->data * d_
                    * exp(-kexp_->data * dist_(j, l));
              }

              break;
            case Vertex::GOAL:
              d_ = ((vertices_[j]->position_ - vertices_[l]->position_).dot(
                  Eigen::Vector3d(EFFJAC.block(3 * j, i, 3, 1)))) / dist_(j, l);
              JAC(cnt, i) = kg_->data * kexp_->data * d_
                  * exp(-kexp_->data * dist_(j, l));
              break;
            default:
              break;
            }
          }
          cnt++;
        }
      }
    }
    return SUCCESS;
  }

  EReturn DMesh::updateDistances()
  {
    dist_.setZero();
    for (int j = 0; j < current_size_; j++)
    {
      for (auto & it : vertices_[j]->toVertices_)
      {
        double tmpd = 0;
        scene_->getCollisionScene()->getDistance(vertices_[j]->name_, it.first,
            tmpd, di_->data);
        if (tmpd < di_->data)
          j < it.second ? dist_(j, it.second) : dist_(it.second, j);
      }
    }
    return SUCCESS;
  }

  EReturn DMesh::updatePositions()
  {
    for (int j = 0; j < current_size_; j++)
    {
      if (!ok(
          scene_->getCollisionScene()->getTranslation(vertices_[j]->name_,
              vertices_[j]->position_)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
    }
    return SUCCESS;
  }

  EReturn DMesh::addGoal(const std::string & name, std::string & toLink)
  {
    if (vertex_map_.find(name) != vertex_map_.end())
    {
      WARNING_NAMED("DMesh", "Add Goal ["<<name<<"] failed, already exists");
      return FAILURE;
    }
    Vertex *tmp = new Vertex(name, Vertex::GOAL, current_size_);
    int eff_index = -1;
    for (int i = 0; i < links_->strings.size(); i++)
      if (links_->strings[i].compare(toLink) == 0)
      {
        tmp->addNewRelation(links_->strings[i], i);
      }
    if (tmp->toVertices_.size() == 0)
    {
      WARNING_NAMED("DMesh",
          "Add goal ["<<name<<"] failed, End-effector link ["<<toLink<<"] does not exist");
      delete tmp;
      return FAILURE;
    }
    vertices_[current_size_] = Vertex_Ptr(tmp);
    current_size_++;
    return SUCCESS;
  }

  EReturn DMesh::addObstacle(const std::string & name)
  {
    if (vertex_map_.find(name) != vertex_map_.end())
    {
      WARNING_NAMED("DMesh", "Add obstacle ["<<name<<"] failed, already exists");
      return FAILURE;
    }
    Vertex *tmp = new Vertex(name, Vertex::OBSTACLE_TO_ALL, current_size_);
    for (int i = 0; i < links_->strings.size(); i++)
      tmp->addNewRelation(links_->strings[i], i);
    vertices_[current_size_] = Vertex_Ptr(tmp);
    current_size_++;
    return SUCCESS;
  }
} //	namespace exotica
