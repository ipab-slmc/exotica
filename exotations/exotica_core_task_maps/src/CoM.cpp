/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
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

#include "exotica_core_task_maps/CoM.h"

REGISTER_TASKMAP_TYPE("CoM", exotica::CoM);

namespace exotica
{
CoM::CoM() = default;
CoM::~CoM() = default;

void CoM::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    double M = mass_.sum();
    if (M == 0.0) return;

    KDL::Vector com;
    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        com += Kinematics[0].Phi(i).p * mass_(i);
        if (debug_)
        {
            com_links_marker_.points[i].x = Kinematics[0].Phi(i).p[0];
            com_links_marker_.points[i].y = Kinematics[0].Phi(i).p[1];
            com_links_marker_.points[i].z = Kinematics[0].Phi(i).p[2];
        }
    }

    com = com / M;
    for (int i = 0; i < dim_; i++) phi(i) = com[i];

    if (debug_)
    {
        com_marker_.pose.position.x = phi(0);
        com_marker_.pose.position.y = phi(1);
        com_marker_.pose.position.z = phi(2);

        com_marker_.header.stamp = com_links_marker_.header.stamp = ros::Time::now();
        com_links_pub_.publish(com_links_marker_);
        com_pub_.publish(com_marker_);
    }
}

void CoM::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    if (J.rows() != dim_ || J.cols() != x.rows()) throw_named("Wrong size of J! " << x.rows());
    J.setZero();
    KDL::Vector com;

    if (Frames.size() > 0)
    {
        double M = mass_.sum();
        if (M == 0.0) return;
        for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
        {
            com += Kinematics[0].Phi(i).p * mass_(i);
            J += mass_(i) / M * Kinematics[0].J(i).data.topRows(dim_);
            if (debug_)
            {
                com_links_marker_.points[i].x = Kinematics[0].Phi(i).p[0];
                com_links_marker_.points[i].y = Kinematics[0].Phi(i).p[1];
                com_links_marker_.points[i].z = Kinematics[0].Phi(i).p[2];
            }
        }
        com = com / M;
    }
    else
    {
        if (debug_) com_links_marker_.points.resize(0);
        double M = 0.0;
        for (std::weak_ptr<KinematicElement> welement : scene_->getKinematicTree().getTree())
        {
            std::shared_ptr<KinematicElement> element = welement.lock();
            if (element->isRobotLink || element->ClosestRobotLink.lock())  // Only for robot links and attached objects
            {
                double mass = element->Segment.getInertia().getMass();
                if (mass > 0)
                {
                    KDL::Frame cog = KDL::Frame(element->Segment.getInertia().getCOG());
                    KDL::Frame com_local = scene_->getKinematicTree().FK(element, cog, nullptr, KDL::Frame());
                    Eigen::MatrixXd Jcom_local = scene_->getKinematicTree().Jacobian(element, cog, nullptr, KDL::Frame());
                    com += com_local.p * mass;
                    J += Jcom_local.topRows(dim_) * mass;
                    M += mass;
                    if (debug_)
                    {
                        geometry_msgs::Point pt;
                        pt.x = com_local.p[0];
                        pt.y = com_local.p[1];
                        pt.z = com_local.p[2];
                        com_links_marker_.points.push_back(pt);
                    }
                }
            }
        }
        if (M == 0.0) return;
        com = com / M;
        J = J / M;
    }
    for (int i = 0; i < dim_; i++) phi(i) = com[i];

    if (debug_)
    {
        com_marker_.pose.position.x = phi(0);
        com_marker_.pose.position.y = phi(1);
        com_marker_.pose.position.z = phi(2);

        com_marker_.header.stamp = com_links_marker_.header.stamp = ros::Time::now();
        com_links_pub_.publish(com_links_marker_);
        com_pub_.publish(com_marker_);
    }
}

int CoM::taskSpaceDim()
{
    return dim_;
}

void CoM::initialize()
{
    enable_z_ = init_.EnableZ;
    if (enable_z_)
        dim_ = 3;
    else
        dim_ = 2;

    if (Frames.size() > 0)
    {
        // NB: This may break if it's an environment frame (#230)
        HIGHLIGHT_NAMED("CoM", "!!! WARNING !!! This may be broken and cause a segfault if you update the scene after initialisation as the weak_ptr in getTreeMap() expire!");
        if (debug_)
            HIGHLIGHT_NAMED("CoM", "Initialisation with " << Frames.size() << " passed into map.");

        mass_.resize(Frames.size());
        for (int i = 0; i < Frames.size(); i++)
        {
            if (Frames[i].FrameBLinkName != "")
            {
                throw_named("Requesting CoM frame with base other than root! '" << Frames[i].FrameALinkName << "'");
            }
            Frames[i].FrameALinkName = scene_->getKinematicTree().getTreeMap()[Frames[i].FrameALinkName].lock()->Segment.getName();
            Frames[i].FrameAOffset.p = scene_->getKinematicTree().getTreeMap()[Frames[i].FrameALinkName].lock()->Segment.getInertia().getCOG();
            mass_(i) = scene_->getKinematicTree().getTreeMap()[Frames[i].FrameALinkName].lock()->Segment.getInertia().getMass();
        }
    }

    if (debug_)
    {
        initialize_debug();
        HIGHLIGHT_NAMED("CoM", "Total model mass: " << mass_.sum() << " kg");
    }
}

void CoM::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    initialize();
}

void CoM::Instantiate(CoMInitializer& init)
{
    init_ = init;
}

void CoM::initialize_debug()
{
    com_links_marker_.points.resize(Frames.size());
    com_links_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    com_links_marker_.color.a = .7;
    com_links_marker_.color.r = 0.5;
    com_links_marker_.color.g = 0;
    com_links_marker_.color.b = 0;
    com_links_marker_.scale.x = com_links_marker_.scale.y = com_links_marker_.scale.z = .02;
    com_links_marker_.action = visualization_msgs::Marker::ADD;

    com_marker_.type = visualization_msgs::Marker::CYLINDER;
    com_marker_.color.a = 1;
    com_marker_.color.r = 1;
    com_marker_.color.g = 0;
    com_marker_.color.b = 0;
    com_marker_.scale.x = com_marker_.scale.y = .15;
    com_marker_.scale.z = .02;
    com_marker_.action = visualization_msgs::Marker::ADD;

    com_links_marker_.header.frame_id = com_marker_.header.frame_id =
        "exotica/" + scene_->getRootFrameName();

    com_links_pub_ = Server::advertise<visualization_msgs::Marker>(object_name_ + "/com_links_marker", 1, true);
    com_pub_ = Server::advertise<visualization_msgs::Marker>(object_name_ + "/com_marker", 1, true);
}
}
