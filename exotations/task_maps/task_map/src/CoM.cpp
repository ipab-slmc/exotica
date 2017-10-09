/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#include "task_map/CoM.h"

REGISTER_TASKMAP_TYPE("CoM", exotica::CoM);

namespace exotica
{
CoM::CoM()
{
}

CoM::~CoM()
{
}

void CoM::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    phi.setZero();
    double M = mass_.sum();
    if (M == 0.0) return;

    KDL::Vector com;
    for (int i = 0; i < Kinematics.Phi.rows(); i++)
    {
        com += Kinematics.Phi(i).p * mass_(i);
        if (debug_)
        {
            com_marker_.points[i].x = Kinematics.Phi(i).p[0];
            com_marker_.points[i].y = Kinematics.Phi(i).p[1];
            com_marker_.points[i].z = Kinematics.Phi(i).p[2];
        }
    }

    com = com / M;
    for (int i = 0; i < dim_; i++) phi(i) = com[i];

    if (debug_)
    {
        COM_marker_.pose.position.x = phi(0);
        COM_marker_.pose.position.y = phi(1);
        COM_marker_.pose.position.z = phi(2);

        COM_marker_.header.stamp = com_marker_.header.stamp = ros::Time::now();
        com_pub_.publish(com_marker_);
        COM_pub_.publish(COM_marker_);
    }
}

void CoM::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    if (J.rows() != dim_ || J.cols() != Kinematics.J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics.J(0).data.cols());
    phi.setZero();
    J.setZero();
    KDL::Vector com;
    double M = mass_.sum();
    if (M == 0.0) return;
    for (int i = 0; i < Kinematics.Phi.rows(); i++)
    {
        com += Kinematics.Phi(i).p * mass_(i);
        J += mass_(i) / M * Kinematics.J(i).data.topRows(dim_);
        if (debug_)
        {
            com_marker_.points[i].x = Kinematics.Phi(i).p[0];
            com_marker_.points[i].y = Kinematics.Phi(i).p[1];
            com_marker_.points[i].z = Kinematics.Phi(i).p[2];
        }
    }
    com = com / M;
    for (int i = 0; i < dim_; i++) phi(i) = com[i];

    if (debug_)
    {
        COM_marker_.pose.position.x = phi(0);
        COM_marker_.pose.position.y = phi(1);
        COM_marker_.pose.position.z = phi(2);

        COM_marker_.header.stamp = com_marker_.header.stamp = ros::Time::now();
        com_pub_.publish(com_marker_);
        COM_pub_.publish(COM_marker_);
    }
}

int CoM::taskSpaceDim()
{
    return dim_;
}

void CoM::Initialize()
{
    enable_z_ = init_.EnableZ;
    if (enable_z_)
        dim_ = 3;
    else
        dim_ = 2;

    if (Frames.size() > 0)
    {
        if (debug_)
            HIGHLIGHT_NAMED("CoM", "Initialisation with " << Frames.size() << " passed into map.");

        mass_.resize(Frames.size());
        for (int i = 0; i < Frames.size(); i++)
        {
            if (Frames[i].FrameBLinkName != "")
            {
                throw_named("Requesting CoM frame with base other than root! '" << Frames[i].FrameALinkName << "'");
            }
            Frames[i].FrameALinkName = scene_->getSolver().getTreeMap()[Frames[i].FrameALinkName]->Segment.getName();
            Frames[i].FrameAOffset.p = scene_->getSolver().getTreeMap()[Frames[i].FrameALinkName]->Segment.getInertia().getCOG();
            mass_(i) = scene_->getSolver().getTreeMap()[Frames[i].FrameALinkName]->Segment.getInertia().getMass();
        }
    }
    else
    {
        int N = scene_->getSolver().getTree().size();
        mass_.resize(N);
        Frames.resize(N);
        if (debug_)
            HIGHLIGHT_NAMED("CoM", "Initialisation for tree of size "
                                       << Frames.size());
        for (int i = 0; i < N; i++)
        {
            Frames[i].FrameALinkName = scene_->getSolver().getTree()[i]->Segment.getName();
            Frames[i].FrameAOffset.p = scene_->getSolver().getTree()[i]->Segment.getInertia().getCOG();
            mass_(i) = scene_->getSolver().getTree()[i]->Segment.getInertia().getMass();
            if (debug_)
                HIGHLIGHT_NAMED("CoM-Initialize",
                                "FrameALinkName: "
                                    << Frames[i].FrameALinkName
                                    << ", mass: " << mass_(i) << ", CoG: ("
                                    << Frames[i].FrameAOffset.p.x() << ", "
                                    << Frames[i].FrameAOffset.p.y() << ", "
                                    << Frames[i].FrameAOffset.p.z() << ")");
        }
    }

    if (debug_)
    {
        InitDebug();
        HIGHLIGHT_NAMED("CoM", "Total model mass: " << mass_.sum() << " kg");
    }
}

void CoM::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void CoM::Instantiate(CoMInitializer& init)
{
    init_ = init;
}

void CoM::InitDebug()
{
    com_marker_.points.resize(Frames.size());
    com_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    com_marker_.color.a = .7;
    com_marker_.color.r = 0.5;
    com_marker_.color.g = 0;
    com_marker_.color.b = 0;
    com_marker_.scale.x = com_marker_.scale.y = com_marker_.scale.z = .02;
    com_marker_.action = visualization_msgs::Marker::ADD;

    COM_marker_.type = visualization_msgs::Marker::CYLINDER;
    COM_marker_.color.a = 1;
    COM_marker_.color.r = 1;
    COM_marker_.color.g = 0;
    COM_marker_.color.b = 0;
    COM_marker_.scale.x = COM_marker_.scale.y = .15;
    COM_marker_.scale.z = .02;
    COM_marker_.action = visualization_msgs::Marker::ADD;

    com_marker_.header.frame_id = COM_marker_.header.frame_id =
        "exotica/" + scene_->getRootFrameName();

    com_pub_ = Server::advertise<visualization_msgs::Marker>(
        object_name_ + "/coms_marker", 1, true);
    COM_pub_ = Server::advertise<visualization_msgs::Marker>(
        object_name_ + "/COM_marker", 1, true);
}
}
