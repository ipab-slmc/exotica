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

#include "IMesh.h"
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

//#define DEBUG_MODE
REGISTER_TASKMAP_TYPE("IMesh", exotica::IMesh);

namespace exotica
{
  IMesh::IMesh()
      : eff_size_(0)
  {
    initialised_ = false;
  }

  IMesh::~IMesh()
  {
    //TODO
  }

  void IMesh::update(Eigen::VectorXdRefConst& x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    if (scene_ != nullptr)
    {
      if (initialised_)
      {
        computeIMesh(t);
      }
      else
      {
        throw_named("Not initialized!");
      }
    }
    else
    {
      throw_named("Not fully initialized!");
    }

  }

  void IMesh::initDebug(std::string ref)
  {
      imesh_mark_.scale.x = 0.005;
      imesh_mark_.color.a = imesh_mark_.color.r = 1;
      imesh_mark_.type = visualization_msgs::Marker::LINE_LIST;
      imesh_mark_.header.frame_id = ref;
      imesh_mark_.ns = getObjectName();
      imesh_mark_pub_ = server_->advertise<visualization_msgs::Marker>(ns_ +"/InteractionMesh", 1, true);
      HIGHLIGHT("InteractionMesh connectivity is published on ROS topic "<<imesh_mark_pub_.getTopic()<<", in reference frame "<<ref);
  }

  void IMesh::Instantiate(IMeshInitializer& init)
  {
      initDebug(init.ReferenceFrame);
      eff_size_ = scene_->getMapSize(object_name_);
      weights_.setOnes(eff_size_, eff_size_);
      initialised_ = true;
  }

  void IMesh::initDerived(tinyxml2::XMLHandle & handle)
  {
    EParam<std_msgs::String> ref;
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("ReferenceFrame");
    server_->registerParam(ns_, tmp_handle, ref);
    if (!tmp_handle.ToElement()) ref->data = "/world";
    initDebug(ref->data);
    eff_size_ = scene_->getMapSize(object_name_);
    weights_.setOnes(eff_size_, eff_size_);
    initialised_ = true;
  }

  void IMesh::debug()
  {
    Eigen::Map<Eigen::MatrixXd> eff_mat(EFFPHI.data(), 3, eff_size_);
    imesh_mark_.points.clear();
    std::vector<geometry_msgs::Point> points(eff_size_);
    for (int i = 0; i < eff_size_; i++)
    {
      points[i].x = eff_mat(0, i);
      points[i].y = eff_mat(1, i);
      points[i].z = eff_mat(2, i);
    }

    for(int i=0;i<eff_size_;i++)
    {
      for(int j=i+1;j<eff_size_;j++)
      {
          imesh_mark_.points.push_back(points[i]);
          imesh_mark_.points.push_back(points[j]);
      }
    }
    imesh_mark_.header.stamp = ros::Time::now();
    imesh_mark_pub_.publish(imesh_mark_);
  }

  void IMesh::destroyDebug()
  {
      imesh_mark_.points.clear();
      imesh_mark_.action = visualization_msgs::Marker::DELETE;
      imesh_mark_.header.stamp = ros::Time::now();
      imesh_mark_pub_.publish(imesh_mark_);
  }

  void IMesh::initialiseManual(std::string name, Server_ptr & server,
      const Scene_ptr & scene_ptr, boost::shared_ptr<PlanningProblem> prob,
      std::vector<std::pair<std::string, std::string> >& params)
  {
    TaskMap::initialiseManual(name, server, scene_ptr, prob, params);
    initDebug("/world");
    eff_size_ = scene_->getMapSize(object_name_);
    weights_.setOnes(eff_size_, eff_size_);
    initialised_ = true;
  }

  void IMesh::taskSpaceDim(int & task_dim)
  {
    LOCK(locker_);
    if (!initialised_)
    {
      throw_named("Not initialized!");
    }
    else
    {
      task_dim = 3 * eff_size_;
    }
  }

  void IMesh::computeLaplace(int t)
  {
    int N = eff_size_;
    dist.resize(N, N);
    dist.setZero();
    wsum.resize(N);
    wsum.setZero();
    int j, l;
    double w;
    /** Compute distance matrix (inverse proportional) */
    for (j = 0; j < N; j++)
    {
      for (l = j + 1; l < N; l++)
      {
        if (!(j >= N && l >= N))
        {
          dist(j, l) = dist(l, j) = sqrt(
              (EFFPHI.segment(j, 3) - EFFPHI.segment(l, 3)).dot(
                  ( EFFPHI.segment(j, 3) - EFFPHI.segment(l, 3))));
        }
      }
    }
    /** Computer weight normaliser */
    for (j = 0; j < N; j++)
    {
      for (l = 0; l < N; l++)
      {
        if (dist(j, l) > 0 && j != l)
        {
          wsum(j) += weights_(j, l) / dist(j, l);
        }
      }
    }
    /** Compute Laplace coordinates */
    for (j = 0; j < N; j++)
    {
      PHI.segment(j, 3) = EFFPHI.segment(j, 3);
      for (l = 0; l < N; l++)
      {
        if (j != l)
        {
          if (dist(j, l) > 0 && wsum(j) > 0)
          {
            w = weights_(j, l) / (dist(j, l) * wsum(j));
            PHI.segment(j, 3) -= EFFPHI.segment(l, 3) * w;
          }
        }
      }
    }
  }

  void IMesh::computeGoalLaplace(const Eigen::VectorXd &x,
      Eigen::VectorXd &goal)
  {
    int t = 0;
    scene_->update(x, t);
    update(x, t);
    computeLaplace(t);
    goal = PHI;
  }

  void IMesh::computeIMesh(int t)
  {
    int M = eff_size_;

    computeLaplace(t);

    if (updateJacobian_)
    {
      double A, _A, Sk, Sl, w, _w;
      int i, j, k, l, N;
      N = EFFJAC.cols();
      Eigen::Vector3d distance, _distance = Eigen::Vector3d::Zero(3, 1);
      for (i = 0; i < N; i++)
      {
        for (j = 0; j < M; j++)
        {
          if (j < M)
          JAC.block(3 * j, i, 3, 1) = EFFJAC.block(3 * j, i, 3, 1);
          for (l = 0; l < M; l++)
          {
            if (j != l)
            {
              if (dist(j, l) > 0 && wsum(j) > 0 && weights_(j, l) > 0)
              {
                A = dist(j, l) * wsum(j);
                w = weights_(j, l) / A;

                _A = 0;
                distance = EFFPHI.segment(j, 3) - EFFPHI.segment(l, 3);
                if (j < M)
                {
                  if (l < M)
                    //Both j and l are points on the robot
                    _distance = EFFJAC.block(3 * j, i, 3, 1)
                        - EFFJAC.block(3 * l, i, 3, 1);
                  else
                    //l is not on the robot
                    _distance = EFFJAC.block(3 * j, i, 3, 1);
                }
                else
                {
                  if (l < M)
                  //j is not on the robot
                    _distance = -EFFJAC.block(3 * l, i, 3, 1);
                }

                Sl = distance.dot(_distance) / dist(j, l);
                for (k = 0; k < M; k++)
                {
                  if (j != k && dist(j, k) > 0 && weights_(j, k) > 0)
                  {
                    distance = EFFPHI.segment(j, 3) - EFFPHI.segment(k, 3);
                    if (j < M)
                    {
                      if (k < M)
                        _distance = EFFJAC.block(3 * j, i, 3, 1)
                            - EFFJAC.block(3 * k, i, 3, 1);
                      else
                        _distance = EFFJAC.block(3 * j, i, 3, 1);
                    }
                    else
                    {
                      if (k < M) _distance = -EFFJAC.block(3 * k, i, 3, 1);
                    }
                    Sk = distance.dot(_distance) / dist(j, k);
                    _A += weights_(j, k) * (Sl * dist(j, k) - Sk * dist(j, l))
                        / (dist(j, k) * dist(j, k));
                  }
                }
                _w = -weights_(j, l) * _A / (A * A);
              }
              else
              {
                _w = 0;
                w = 0;
              }
              if (l < M)
                JAC.block(3 * j, i, 3, 1) -= EFFPHI.segment(l, 3) * _w
                    + EFFJAC.block(3 * l, i, 3, 1) * w;
              else
                JAC.block(3 * j, i, 3, 1) -= EFFPHI.segment(l, 3) * _w;
            }
          }
        }
      }
    }
  }

  void IMesh::setWeight(int i, int j, double weight)
  {
    uint M = weights_.cols();
    if (i < 0 || i >= M || j < 0 || j >= M)
    {
      throw_named("Invalid weight element (" << i << "," << j
          << "). Weight matrix " << M << "x" << M );
    }
    if (weight < 0)
    {
      throw_named("Invalid weight: " << weight );
    }
    weights_(i, j) = weight;
  }

  void IMesh::setWeights(const Eigen::MatrixXd & weights)
  {
    uint M = weights_.cols();
    if (weights.rows() != M || weights.cols() != M)
    {
      throw_named("Invalid weight matrix (" << weights.rows() << "X"
          << weights.cols() << "). Has to be" << M << "x" << M );
    }
    weights_ = weights;
  }
}
