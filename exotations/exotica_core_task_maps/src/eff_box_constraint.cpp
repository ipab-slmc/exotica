//
// Copyright (c) 2019, Christopher E. Mower
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

#include <exotica_core_task_maps/eff_box_constraint.h>

REGISTER_TASKMAP_TYPE("EffBoxConstraint", exotica::EffBoxConstraint);

namespace exotica
{
void EffBoxConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < n_effs_; ++i)
    {
        // Setup
        const int eff_id = 6 * i;
        Eigen::Vector3d e = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);

        // Compute phi
        phi.segment(eff_id, 3) = e - eff_upper;
        phi.segment(eff_id + 3, 3) = eff_lower - e;
    }
}

void EffBoxConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != TaskSpaceDim() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < n_effs_; ++i)
    {
        // Setup
        const int eff_id = 6 * i;
        Eigen::Vector3d e = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);

        // Compute phi
        phi.segment(eff_id, 3) = e - eff_upper;
        phi.segment(eff_id + 3, 3) = eff_lower - e;

        // Compute jacobian
        jacobian.block(eff_id, 0, 3, jacobian.cols()) = kinematics[0].jacobian(i).data.topRows<3>();
        jacobian.block(eff_id + 3, 0, 3, jacobian.cols()) = -kinematics[0].jacobian(i).data.topRows<3>();
    }
}

void EffBoxConstraint::Instantiate(const EffBoxConstraintInitializer& init)
{
    parameters_ = init;

    if (init.XLim[0] > init.XLim[1]) ThrowPretty("Specify XLim using lower then upper.");
    if (init.YLim[0] > init.YLim[1]) ThrowPretty("Specify YLim using lower then upper.");
    if (init.ZLim[0] > init.ZLim[1]) ThrowPretty("Specify ZLim using lower then upper.");

    eff_lower[0] = init.XLim[0];
    eff_upper[0] = init.XLim[1];
    eff_lower[1] = init.YLim[0];
    eff_upper[1] = init.YLim[1];
    eff_lower[2] = init.ZLim[0];
    eff_upper[2] = init.ZLim[1];

    n_effs_ = frames_.size();
}

int EffBoxConstraint::TaskSpaceDim()
{
    return 6 * n_effs_;
}
}  // namespace exotica
