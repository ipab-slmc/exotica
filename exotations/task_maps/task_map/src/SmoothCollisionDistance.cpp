/*
 *      Author: Wolfgang Merkt
 *
 * Copyright (c) 2017, Wolfgang Merkt
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

#include "task_map/SmoothCollisionDistance.h"

REGISTER_TASKMAP_TYPE("SmoothCollisionDistance",
                      exotica::SmoothCollisionDistance);

namespace exotica
{
SmoothCollisionDistance::SmoothCollisionDistance() {}
void SmoothCollisionDistance::update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi)
{
    throw_pretty("I didn't implement that yet");
}

void SmoothCollisionDistance::update(Eigen::VectorXdRefConst x,
                                     Eigen::VectorXdRef phi,
                                     Eigen::MatrixXdRef J)
{
    if (phi.rows() != dim_) throw_named("Wrong size of phi!");
    if (!scene_->alwaysUpdatesCollisionScene())
        cscene_->updateCollisionObjectTransforms();

    phi.setZero();
    J.setZero();

    // Get all world collision links, then iterate through them
    for (unsigned int i = 0; i < dim_; i++)
    {
        std::vector<CollisionProxy> proxies =
            cscene_->getCollisionDistance(robotLinks[i]);
        for (const auto& proxy : proxies)
        {
            if (proxy.distance < margin_)
            {
                // Cost
                phi(i) += std::pow((1 - proxy.distance / margin_), linear_ ? 1 : 2);
                const double& d = phi(i);

                // Jacobian
                KDL::Frame arel = KDL::Frame(proxy.e1->Frame.Inverse(KDL::Vector(
                    proxy.contact1(0), proxy.contact1(1), proxy.contact1(2))));
                KDL::Frame brel = KDL::Frame(proxy.e2->Frame.Inverse(KDL::Vector(
                    proxy.contact2(0), proxy.contact2(1), proxy.contact2(2))));

                if (!linear_)
                {
                    Eigen::MatrixXd tmpJ = scene_->getSolver().Jacobian(
                        proxy.e1, arel, nullptr, KDL::Frame());
                    J.row(i) -= (2. * d) / margin_ * (proxy.normal1.transpose() * tmpJ);
                    tmpJ = scene_->getSolver().Jacobian(proxy.e2, brel, nullptr,
                                                        KDL::Frame());
                    J.row(i) += (2. * d) / margin_ * (proxy.normal1.transpose() * tmpJ);
                }
                else
                {
                    Eigen::MatrixXd tmpJ = scene_->getSolver().Jacobian(
                        proxy.e1, arel, nullptr, KDL::Frame());
                    J.row(i) -= 1 / margin_ * (proxy.normal1.transpose() * tmpJ);
                    tmpJ = scene_->getSolver().Jacobian(proxy.e2, brel, nullptr,
                                                        KDL::Frame());
                    J.row(i) += 1 / margin_ * (proxy.normal1.transpose() * tmpJ);
                }
            }
        }
    }
}

void SmoothCollisionDistance::Instantiate(
    SmoothCollisionDistanceInitializer& init)
{
    init_ = init;
}

void SmoothCollisionDistance::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void SmoothCollisionDistance::Initialize()
{
    cscene_ = scene_->getCollisionScene();
    margin_ = init_.Margin;
    linear_ = init_.Linear;

    HIGHLIGHT_NAMED("Smooth Collision Distance",
                    "Margin: " << margin_ << "\t Linear: " << linear_);

    // Get names of all controlled joints and their corresponding child links
    for (const auto& element : scene_->getSolver().getTree())
    {
        bool isControlled = scene_->getSolver().IsControlled(element) >= 0;
        if (isControlled)
        {
            robotLinks.push_back(element->Segment.getName());
            robotKinematicElements.push_back(element);
        }

        // if (debug_)
        //   HIGHLIGHT_NAMED("Smooth Collision Distance",
        //                   "Joint=" << element->Segment.getJoint().getName()
        //                            << " - Child Link=" <<
        //                            element->Segment.getName()
        //                            << " - Controlled: " << isControlled);
    }

    dim_ = static_cast<int>(scene_->getSolver().getJointNames().size());
    if (dim_ != robotLinks.size())
        throw_pretty(
            "Number of children links does not match number of controlled joints.");
}

int SmoothCollisionDistance::taskSpaceDim() { return dim_; }
}
