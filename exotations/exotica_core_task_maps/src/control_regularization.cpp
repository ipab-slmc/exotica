//
// Copyright (c) 2020, University of Oxford
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_core_task_maps/control_regularization.h>

REGISTER_TASKMAP_TYPE("ControlRegularization", exotica::ControlRegularization);

namespace exotica
{
// As this is a control-task-map, the configuration updates are all empty.
void ControlRegularization::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef /*phi*/)
{
}

void ControlRegularization::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef /*phi*/, Eigen::MatrixXdRef /*jacobian*/)
{
}

void ControlRegularization::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef /*phi*/, Eigen::MatrixXdRef /*jacobian*/, HessianRef /*hessian*/)
{
}

// Dynamic update methods
void ControlRegularization::Update(Eigen::VectorXdRefConst /*x*/, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi)
{
    if (phi.rows() != static_cast<int>(joint_map_.size())) ThrowNamed("Wrong size of Phi!");
    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(i) = u(joint_map_[i]) - joint_ref_(i);
    }
}

void ControlRegularization::Update(Eigen::VectorXdRefConst /*x*/, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef /*dphi_dx*/, Eigen::MatrixXdRef dphi_du)
{
    if (phi.rows() != static_cast<int>(joint_map_.size())) ThrowNamed("Wrong size of Phi!");
    if (dphi_du.rows() != static_cast<int>(joint_map_.size()) || dphi_du.cols() != num_controlled_joints_) ThrowNamed("Wrong size of jacobian! " << num_controlled_joints_);
    for (std::size_t i = 0; i < joint_map_.size(); ++i)
    {
        phi(i) = u(joint_map_[i]) - joint_ref_(i);
        dphi_du(i, joint_map_[i]) = 1.0;
    }
}

void ControlRegularization::Update(Eigen::VectorXdRefConst /*x*/, Eigen::VectorXdRefConst u, Eigen::VectorXdRef phi, Eigen::MatrixXdRef /*dphi_dx*/, Eigen::MatrixXdRef dphi_du, HessianRef /*ddphi_ddx*/, HessianRef ddphi_ddu, HessianRef /*ddphi_dxdu*/)
{
    Eigen::VectorXd x_none(1);
    Eigen::MatrixXd dphi_dx_none(1, 1);
    Update(x_none, u, phi, dphi_dx_none, dphi_du);
}

void ControlRegularization::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void ControlRegularization::Initialize()
{
    num_controlled_joints_ = scene_->get_num_controls();

    if (num_controlled_joints_ == 0) ThrowPretty("Not a dynamic scene? Number of controls is 0.");

    if (parameters_.JointMap.rows() > 0)
    {
        joint_map_.resize(parameters_.JointMap.rows());
        for (int i = 0; i < parameters_.JointMap.rows(); ++i)
        {
            joint_map_[i] = parameters_.JointMap(i);
        }
    }
    else
    {
        joint_map_.resize(num_controlled_joints_);
        for (int i = 0; i < num_controlled_joints_; ++i)
        {
            joint_map_[i] = i;
        }
    }

    if (parameters_.JointRef.rows() > 0)
    {
        joint_ref_ = parameters_.JointRef;
        if (joint_ref_.rows() != static_cast<int>(joint_map_.size())) ThrowNamed("Invalid joint reference size! Expecting " << joint_map_.size() << " but received " << joint_ref_.rows());
    }
    else
    {
        joint_ref_ = Eigen::VectorXd::Zero(joint_map_.size());
    }
}

int ControlRegularization::TaskSpaceDim()
{
    return joint_map_.size();
}

const std::vector<int>& ControlRegularization::get_joint_map() const
{
    return joint_map_;
}

const Eigen::VectorXd& ControlRegularization::get_joint_ref() const
{
    return joint_ref_;
}

}  // namespace exotica
