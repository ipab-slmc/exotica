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

#include "SweepFlux.h"

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

//#define DEBUG_MODE
REGISTER_TASKMAP_TYPE("SweepFlux", exotica::SweepFlux);
namespace exotica
{

  SweepFlux::SweepFlux()
      : initialised_(false), T_(0), init_int_(false), init_vis_(false)
  {

  }

  void SweepFlux::setTimeSteps(const int T)
  {
    TaskMap::setTimeSteps(T);
    int n, m;
    n = scene_->getNumJoints();
    taskSpaceDim(m);
    T_ = T;
    Verts_ = Eigen::VectorXd::Zero(scene_->getMapSize(object_name_) * T_ * 3);

    if (T_ < 2)
    {
      throw_named("Too few timesteps!");
    }
    if (scene_->getMapSize(object_name_) < 2)
    {
      throw_named("Wrong number of end-effectors!");
    }
    TrisStride_ = (scene_->getMapSize(object_name_) - 1) * 2
        + (capTop_->data ? 1 : 0) + (capBottom_->data ? 1 : 0);
    if (capEnds_->data)
    {
      Tris_.resize(TrisStride_ * T_ * 3);
      TriFlux_.resize(TrisStride_ * T_);
      FluxQ_.resize(TrisQ_.rows() / 3 * T_);
    }
    else
    {
      Tris_.resize(TrisStride_ * (T_ - 1) * 3);
      TriFlux_.resize(TrisStride_ * (T_ - 1));
      FluxQ_.resize(TrisQ_.rows() / 3 * (T_));
    }
    FluxQ_.setZero();
    int a, b, c, d;
    for (int t = 0; t < (capEnds_->data ? T_ : (T_ - 1)); t++)
    {
      if (capTop_->data)
      {
        a = t * (scene_->getMapSize(object_name_));
        b = ((t + 1) % T_) * (scene_->getMapSize(object_name_));
        c = 0;
        Tris_(t * TrisStride_ * 3) = a;
        Tris_(t * TrisStride_ * 3 + 1) = b;
        Tris_(t * TrisStride_ * 3 + 2) = c;
      }
      if (capBottom_->data)
      {
        b = t * (scene_->getMapSize(object_name_))
            + scene_->getMapSize(object_name_) - 1;
        a = ((t + 1) % T_) * (scene_->getMapSize(object_name_))
            + scene_->getMapSize(object_name_) - 1;
        c = scene_->getMapSize(object_name_) - 1;
        Tris_(
            ((capTop_->data ? 1 : 0) + t * TrisStride_
                + (scene_->getMapSize(object_name_) - 1) * 2) * 3) = a;
        Tris_(
            ((capTop_->data ? 1 : 0) + t * TrisStride_
                + (scene_->getMapSize(object_name_) - 1) * 2) * 3 + 1) = b;
        Tris_(
            ((capTop_->data ? 1 : 0) + t * TrisStride_
                + (scene_->getMapSize(object_name_) - 1) * 2) * 3 + 2) = c;
      }
      for (int i = 0; i < scene_->getMapSize(object_name_) - 1; i++)
      {
        a = t * (scene_->getMapSize(object_name_)) + i;
        b = t * (scene_->getMapSize(object_name_)) + i + 1;
        c = ((t + 1) % T_) * (scene_->getMapSize(object_name_)) + i;
        d = ((t + 1) % T_) * (scene_->getMapSize(object_name_)) + i + 1;
        Tris_(((capTop_->data ? 1 : 0) + t * TrisStride_ + i * 2 + 0) * 3) = a;
        Tris_(((capTop_->data ? 1 : 0) + t * TrisStride_ + i * 2 + 0) * 3 + 1) =
            b;
        Tris_(((capTop_->data ? 1 : 0) + t * TrisStride_ + i * 2 + 0) * 3 + 2) =
            c;
        Tris_(((capTop_->data ? 1 : 0) + t * TrisStride_ + i * 2 + 1) * 3) = c;
        Tris_(((capTop_->data ? 1 : 0) + t * TrisStride_ + i * 2 + 1) * 3 + 1) =
            b;
        Tris_(((capTop_->data ? 1 : 0) + t * TrisStride_ + i * 2 + 1) * 3 + 2) =
            d;
      }
    }
    //ROS_WARN_STREAM("\n"<<Tris_);
    TriFlux_.setZero();
    initialised_ = true;

  }

  SweepFlux::~SweepFlux()
  {

  }

  void SweepFlux::initVis(ros::NodeHandle nh, std::string topic)
  {
    nh_ = nh;
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>(topic, 0);
    init_vis_ = true;
  }

  void SweepFlux::doVis()
  {

    visualization_msgs::Marker marker;
    marker.header.frame_id = scene_->getRootName();
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = "SweepFlux - " + getObjectName();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.points.resize(TrisQ_.rows() + Tris_.rows() * 2);
    marker.colors.resize(TrisQ_.rows() + Tris_.rows() * 2);
    //ROS_WARN_STREAM(TrisQ_.rows()<<"x"<<TrisQ_.cols()<<"\n"<<VertsQ_.rows()<<"x"<<VertsQ_.cols());
    double fmax = -1e100, fmin = 1e100;
    for (int i = 0; i < TrisQ_.rows() / 3; i++)
    {
      Eigen::Vector3d tmp1 = VertsQ_.middleRows(TrisQ_(i * 3) * 3, 3);
      Eigen::Vector3d tmp2 = VertsQ_.middleRows(TrisQ_(i * 3 + 1) * 3, 3);
      Eigen::Vector3d tmp3 = VertsQ_.middleRows(TrisQ_(i * 3 + 2) * 3, 3);
      Eigen::Vector3d tmp = (tmp2 - tmp1).cross(tmp3 - tmp1);
      Eigen::Vector3d zz;
      zz << 0.0, 0.0, 1.0;
      tmp = tmp / tmp.norm();
      if (tmp(0) != tmp(0))
      {
        tmp.setZero();
      }
      else
      {
        tmp(0) = tmp.dot(zz);
      }
      double flx = 0.0;
      for (int j = 0; j < T_; j++)
      {
        flx += FluxQ_[i + j * (TrisQ_.rows() / 3)];
      }
      fmin = std::min(fmin, flx);
      fmax = std::max(fmax, flx);
      for (int j = 0; j < 3; j++)
      {
        marker.points[i * 3 + j].x = VertsQ_(TrisQ_(i * 3 + j) * 3);
        marker.points[i * 3 + j].y = VertsQ_(TrisQ_(i * 3 + j) * 3 + 1);
        marker.points[i * 3 + j].z = VertsQ_(TrisQ_(i * 3 + j) * 3 + 2);
        marker.colors[i * 3 + j].r = flx;
        marker.colors[i * 3 + j].g = 0.0;
        marker.colors[i * 3 + j].b = 0.0;
        marker.colors[i * 3 + j].a = 1.0;
      }
    }
    if (fabs(fmin) < fabs(fmax))
    {
      fmin = fmin / fabs(fmin) * fabs(fmax);
    }
    else
    {
      fmax = fmax / fabs(fmax) * fabs(fmin);
    }
    for (int i = 0; i < TrisQ_.rows() / 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        double flx = marker.colors[i * 3 + j].r;
        marker.colors[i * 3 + j].r = 0.0;
        if (flx > 0)
        {
          marker.colors[i * 3 + j].r = flx / fmax;
        }
        if (flx < 0)
        {
          marker.colors[i * 3 + j].b = flx / fmin;
        }
      }
    }
    for (int i = 0; i < Tris_.rows() / 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        marker.points[TrisQ_.rows() + i * 3 + j].x = Verts_(
            Tris_(i * 3 + j) * 3);
        marker.points[TrisQ_.rows() + i * 3 + j].y = Verts_(
            Tris_(i * 3 + j) * 3 + 1);
        marker.points[TrisQ_.rows() + i * 3 + j].z = Verts_(
            Tris_(i * 3 + j) * 3 + 2);
        marker.colors[TrisQ_.rows() + i * 3 + j].r = 0.2;
        marker.colors[TrisQ_.rows() + i * 3 + j].g = 1.0
            - ((i / TrisStride_) % 2 == 0 ? 0.0 : 0.3);
        marker.colors[TrisQ_.rows() + i * 3 + j].b = 0.2;
        marker.colors[TrisQ_.rows() + i * 3 + j].a = 1.0;
      }
    }
    for (int i = 0; i < Tris_.rows() / 3; i++)
    {
      Eigen::Vector3d tmp1 = Verts_.middleRows(Tris_(i * 3) * 3, 3);
      Eigen::Vector3d tmp2 = Verts_.middleRows(Tris_(i * 3 + 1) * 3, 3);
      Eigen::Vector3d tmp3 = Verts_.middleRows(Tris_(i * 3 + 2) * 3, 3);
      Eigen::Vector3d tmp = (tmp2 - tmp1).cross(tmp3 - tmp1);
      tmp = tmp / tmp.norm() * 0.001;
      if (tmp(0) != tmp(0))
      {
        tmp.setZero();
      }
      for (int j = 0; j < 3; j++)
      {
        marker.points[TrisQ_.rows() + i * 3 + j + Tris_.rows()].x = Verts_(
            Tris_(i * 3 + j) * 3) - tmp(0);
        marker.points[TrisQ_.rows() + i * 3 + j + Tris_.rows()].y = Verts_(
            Tris_(i * 3 + j) * 3 + 1) - tmp(1);
        marker.points[TrisQ_.rows() + i * 3 + j + Tris_.rows()].z = Verts_(
            Tris_(i * 3 + j) * 3 + 2) - tmp(2);
        marker.colors[TrisQ_.rows() + i * 3 + j + Tris_.rows()].r = 0.2;
        marker.colors[TrisQ_.rows() + i * 3 + j + Tris_.rows()].g = 0.2;
        marker.colors[TrisQ_.rows() + i * 3 + j + Tris_.rows()].b = 1.0
            - ((i / TrisStride_) % 2 == 0 ? 0.0 : 0.3);
        marker.colors[TrisQ_.rows() + i * 3 + j + Tris_.rows()].a = 1.0;
      }
    }
    vis_pub_.publish(marker);

  }

  void SweepFlux::update(Eigen::VectorXdRefConst x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    if (scene_ != nullptr)
    {
      if (initialised_)
      {
        computeFlux(t);
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

  void SweepFlux::taskSpaceDim(int & task_dim)
  {
    task_dim = 1;
  }

  void SweepFlux::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("ObjectMesh");
    server_->registerRosParam<std::string>(ns_, tmp_handle, obj_file_);
    loadOBJ(*obj_file_, TrisQ_, VertsQ_orig_);

    tmp_handle = handle.FirstChildElement("CapTop");
    server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, capTop_);
    tmp_handle = handle.FirstChildElement("CapBottom");
    server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, capBottom_);
    tmp_handle = handle.FirstChildElement("CapEnds");
    server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, capEnds_);
    //tmp_handle = handle.FirstChildElement("ObjectPose");
    //server_->registerParam<geometry_msgs::PoseStamped>(ns_, tmp_handle, obj_pose_);
    Eigen::Affine3d val;
    transform(val);

    init_int_ = true;
  }

  void SweepFlux::transform(Eigen::Affine3d& val)
  {
    qTransform_ = val;
    VertsQ_.resize(VertsQ_orig_.rows());
    for (int i = 0; i < VertsQ_.rows() / 3; i++)
    {
      VertsQ_.segment(i * 3, 3) = qTransform_
          * ((Eigen::Vector3d) VertsQ_orig_.segment(i * 3, 3));
    }
  }

  void SweepFlux::computeFlux(const int t)
  {
    Verts_.segment(t * scene_->getMapSize(object_name_) * 3,
        scene_->getMapSize(object_name_) * 3) = EFFPHI;
    VertJ_ = Eigen::MatrixXd::Zero(scene_->getMapSize(object_name_) * 3 * 3,
        EFFJAC.cols());

    if (t == 0)
    {
      VertJ_.block(0, 0, scene_->getMapSize(object_name_) * 3, EFFJAC.cols()) =
          EFFJAC;
    }
    else
    {
      VertJ_.block(scene_->getMapSize(object_name_) * 3, 0,
          scene_->getMapSize(object_name_) * 3, EFFJAC.cols()) = EFFJAC;
    }

    int tri_start = (t + (t == 0 ? 0 : -1)) * TrisStride_;
    int tri_length = ((t == -1 || ((t == T_ - 1) && capEnds_->data)) ? 2 : 1)
        * TrisStride_;

    FluxTriangleTriangle(&Tris_.data()[tri_start * 3], Verts_.data(),
        VertJ_.data(), Tris_.data(), TrisQ_.data(), VertsQ_.data(),
        &TriFlux_.data()[tri_start], JAC.data(), tri_length, VertJ_.rows(),
        VertJ_.cols(), TrisQ_.rows() / 3,
        &FluxQ_.data()[(t + (t == 0 ? 0 : -1)) * (TrisQ_.rows() / 3)]);

    PHI(0) = TriFlux_.segment(tri_start, tri_length).sum();
  }

  void SweepFlux::FluxTriangleTriangle(int* Tris, double* Verts, double* VertJ,
      int* TrisJ, int* TrisQ, double* VertsQ, double* Flux, double* FluxJ,
      int nTris, int nTrisJ, int n, int nTrisQ, double* FluxQ)
  {
    memset(Flux, 0, sizeof(double) * nTris);
    memset(FluxJ, 0, sizeof(double) * n);
    double *a, *b, *c, *d, *e, *f, tmp[3], m1[3], m2[3], m3[3], m4[3], *aJ, *bJ,
        *cJ, *tmpFluxJ, area, f0, f1, f2, f3;
    aJ = new double[3 * n];
    bJ = new double[3 * n];
    cJ = new double[3 * n];
    tmpFluxJ = new double[n * 4];

    for (int j = 0; j < nTrisQ; j++)
    {
      FluxQ[j] = 0.0;
      d = &VertsQ[TrisQ[j * 3] * 3];
      e = &VertsQ[TrisQ[j * 3 + 1] * 3];
      f = &VertsQ[TrisQ[j * 3 + 2] * 3];
      cross(e, d, f, d, tmp);
      area = norm(tmp) * 0.5;
      for (int i = 0; i < nTris; i++)
      {
        a = &Verts[Tris[i * 3] * 3];
        b = &Verts[Tris[i * 3 + 1] * 3];
        c = &Verts[Tris[i * 3 + 2] * 3];

        for (int k = 0; k < n; k++)
        {
          aJ[k * 3] = VertJ[TrisJ[i * 3] * 3 + k * nTrisJ];
          aJ[k * 3 + 1] = VertJ[TrisJ[i * 3] * 3 + k * nTrisJ + 1];
          aJ[k * 3 + 2] = VertJ[TrisJ[i * 3] * 3 + k * nTrisJ + 2];

          bJ[k * 3] = VertJ[TrisJ[i * 3 + 1] * 3 + k * nTrisJ];
          bJ[k * 3 + 1] = VertJ[TrisJ[i * 3 + 1] * 3 + k * nTrisJ + 1];
          bJ[k * 3 + 2] = VertJ[TrisJ[i * 3 + 1] * 3 + k * nTrisJ + 2];

          cJ[k * 3] = VertJ[TrisJ[i * 3 + 2] * 3 + k * nTrisJ];
          cJ[k * 3 + 1] = VertJ[TrisJ[i * 3 + 2] * 3 + k * nTrisJ + 1];
          cJ[k * 3 + 2] = VertJ[TrisJ[i * 3 + 2] * 3 + k * nTrisJ + 2];
        }

        for (int k = 0; k < 3; k++)
        {

          m1[k] = (4.0 * d[k] + e[k] + f[k]) / 6.0;
          m2[k] = (d[k] + 4.0 * e[k] + f[k]) / 6.0;
          m3[k] = (d[k] + e[k] + 4.0 * f[k]) / 6.0;
          m4[k] = (d[k] + e[k] + f[k]) / 3.0;
        }

        FluxPointTriangle(m1, a, b, c, aJ, bJ, cJ, &f0, &tmpFluxJ[0], n);
        FluxPointTriangle(m2, a, b, c, aJ, bJ, cJ, &f1, &tmpFluxJ[n], n);
        FluxPointTriangle(m3, a, b, c, aJ, bJ, cJ, &f2, &tmpFluxJ[n * 2], n);
        FluxPointTriangle(m4, a, b, c, aJ, bJ, cJ, &f3, &tmpFluxJ[n * 3], n);
        Flux[i] += (f0 + f1 + f2 + f3) * area;
        FluxQ[j] += (f0 + f1 + f2 + f3) * area;

        for (int k = 0; k < n; k++)
          FluxJ[k] += (tmpFluxJ[k] + tmpFluxJ[k + n] + tmpFluxJ[k + 2 * n]
              + tmpFluxJ[k + 3 * n]) * area;
      }
    }

    delete[] aJ;
    delete[] bJ;
    delete[] cJ;
    delete[] tmpFluxJ;
  }

}
