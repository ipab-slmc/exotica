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

#include "SphereCollision.h"

REGISTER_TASKMAP_TYPE("SphereCollision", exotica::SphereCollision);

namespace exotica
{
  SphereCollision::SphereCollision()
  {
    //!< Empty constructor
  }

  void SphereCollision::Instantiate(SphereCollisionInitializer& init)
  {
     eps = 1.0/init.Precision;
     for(int i=0;i<init.EndEffector.size();i++)
     {
         SphereInitializer sphere(init.EndEffector[i]);
         groups[sphere.Group].push_back(i);
         radiuses.push_back(sphere.Radius);
         visualization_msgs::Marker mrk;
         mrk.action = visualization_msgs::Marker::ADD;
         mrk.header.frame_id = init.ReferenceFrame;
         mrk.id = i;
         mrk.type = visualization_msgs::Marker::SPHERE;
         mrk.scale.x = mrk.scale.y = mrk.scale.z = sphere.Radius*2;
         debug_msg.markers.push_back(mrk);
     }
     for(auto& it : groups)
     {
         std_msgs::ColorRGBA col = randomColor();
         col.a = init.Alpha;
         for(int i : it.second)
         {
             debug_msg.markers[i].color = col;
             debug_msg.markers[i].ns = it.first;
         }
     }
     debug = init.Debug;
     debug_pub = server_->advertise<visualization_msgs::MarkerArray>(ns_ +"/CollisionSpheres", 1, true);
  }

  double SphereCollision::distance(Eigen::VectorXdRefConst effA, Eigen::VectorXdRefConst effB, double rA, double rB)
  {
    return 1.0/(1.0 + exp(5.0 * eps * (sqrt((effA-effB).adjoint()*(effA-effB))-rA-rB)));
  }

  Eigen::VectorXd SphereCollision::Jacobian(Eigen::VectorXdRefConst effA, Eigen::VectorXdRefConst effB, Eigen::MatrixXdRefConst jacA, Eigen::MatrixXdRefConst jacB, double rA, double rB)
  {
    int n = jacA.cols();
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(n);
    for(int i=0;i<n;i++)
    {
        ret(i) = -(double)((jacA.col(i)-jacB.col(i)).adjoint()*(effA-effB)) / sqrt((effA-effB).adjoint()*(effA-effB));
    }
    return ret;
  }

  void SphereCollision::update(Eigen::VectorXdRefConst x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    PHI.setZero();
    if (updateJacobian_)
    {
      JAC.setZero();
    }

    int i=0;
    auto Aend = groups.end()--;
    auto Bend = groups.end();
    int phiI = 0;
    for(auto A=groups.begin(); A!=Aend; A++)
    {
        for(auto B=std::next(A); B!=Bend; B++)
        {
            for(int ii=0;ii<A->second.size();ii++)
            {
                for(int jj=0;jj<B->second.size();jj++)
                {
                    int i = A->second[ii];
                    int j = B->second[jj];
                    PHI(phiI) += distance(EFFPHI.segment(i*3,3), EFFPHI.segment(j*3,3), radiuses[i], radiuses[j]);
                    if (updateJacobian_)
                    {
                        JAC.row(phiI) += Jacobian(EFFPHI.segment(i*3,3), EFFPHI.segment(j*3,3), EFFJAC.middleRows(i*3,3), EFFJAC.middleRows(j*3,3), radiuses[i], radiuses[j]);
                    }
                }
            }
            phiI++;
        }
    }

    if(debug)
    {
        for(int i=0;i<debug_msg.markers.size();i++)
        {
            debug_msg.markers[i].pose.position.x = EFFPHI(i*3);
            debug_msg.markers[i].pose.position.y = EFFPHI(i*3+1);
            debug_msg.markers[i].pose.position.z = EFFPHI(i*3+2);
        }
        debug_pub.publish(debug_msg);
    }
  }

  void SphereCollision::taskSpaceDim(int & task_dim)
  {
    if (!scene_)
    {
      throw_named("Kinematic scene has not been initialized!");
    }
    else
    {
      task_dim = groups.size()*(groups.size()-1)/2;
    }
  }
}
