//
// Copyright (c) 2018, University of Edinburgh
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

#include <exotica_core_task_maps/look_at.h>

REGISTER_TASKMAP_TYPE("LookAt", exotica::LookAt);

namespace exotica
{
LookAt::LookAt() = default;
LookAt::~LookAt() = default;

Eigen::Vector3d LookAt::get_look_at_target_in_world(const int& i)
{
    if (i >= n_end_effs_ || i < 0) ThrowPretty("Out of bounds, got " << i << " but expected less than " << n_end_effs_);
    return Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(n_end_effs_ * i + 2).p.data);
}

void LookAt::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < n_end_effs_; ++i)
    {
        const int end_effector_id = i * n_end_effs_;

        // Get EffPoint and LookAtTarget
        Eigen::Vector3d c = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(end_effector_id).p.data);      // EffPoint | Eff frame
        Eigen::Vector3d p = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(end_effector_id + 1).p.data);  // LookAtTarget | Eff frame

        // Compute orthogonal orthogonal projection a onto line e->c
        double alpha = p.dot(c) / c.squaredNorm();
        Eigen::Vector3d a = alpha * c;

        // Set Phi
        phi.segment<3>(end_effector_id) = a - p;
    }
}

void LookAt::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != TaskSpaceDim() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < n_end_effs_; ++i)
    {
        const int end_effector_id = i * n_end_effs_;

        // Get EffPoint and LookAtTarget
        Eigen::Vector3d c = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(end_effector_id).p.data);      // EffPoint | Eff frame
        Eigen::Vector3d p = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(end_effector_id + 1).p.data);  // LookAtTarget | Eff frame

        // Compute orthogonal orthogonal projection a onto line e->c
        double c_squared_norm = c.squaredNorm();
        double alpha = p.dot(c) / c_squared_norm;
        Eigen::Vector3d a = alpha * c;

        // Set Phi
        phi.segment<3>(end_effector_id) = a - p;

        // Compute jacobian
        for (int j = 0; j < jacobian.cols(); ++j)
        {
            Eigen::Vector3d pd = kinematics[0].jacobian[end_effector_id + 1].data.topRows<3>().col(j);
            double alphad = c.dot(pd) / c_squared_norm;
            jacobian.middleRows<3>(end_effector_id).col(j) = alphad * c - pd;
        }
    }
}

void LookAt::Instantiate(LookAtInitializer& init)
{
    // Error check
    if (frames_.size() % 3 != 0) ThrowNamed("Three frames are required for each end-effector!");

    // Init private variables
    n_end_effs_ = frames_.size() / 3;
    n_ = frames_.size();

    // Verify that the second and third frames are the same
    for (int i = 0; i < n_end_effs_; ++i)
    {
        if (frames_[i + 1].frame_A_link_name != frames_[i + 2].frame_A_link_name)
        {
            ThrowPretty("The second and third links (LookAtTarget) need to be the same! Got: " << frames_[i + 1].frame_A_link_name << " and " << frames_[i + 2].frame_A_link_name);
        }
    }
}

int LookAt::TaskSpaceDim()
{
    return n_;
}
}
