//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <exotica_core_task_maps/distance.h>

REGISTER_TASKMAP_TYPE("Distance", exotica::Distance);

namespace exotica
{
void Distance::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i) = kinematics[0].Phi(i).p.Norm();
    }
}

void Distance::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        phi(i) = kinematics[0].Phi(i).p.Norm();
        jacobian.row(i) = (kinematics[0].Phi(i).p[0] * kinematics[0].jacobian[i].data.row(0) + kinematics[0].Phi(i).p[1] * kinematics[0].jacobian[i].data.row(1) + kinematics[0].Phi(i).p[2] * kinematics[0].jacobian[i].data.row(2)) / phi(i);
    }
}

void Distance::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        const double& p_x = kinematics[0].Phi(i).p[0];
        const double& p_y = kinematics[0].Phi(i).p[1];
        const double& p_z = kinematics[0].Phi(i).p[2];

        const Eigen::RowVectorXd& dp_x_dq = kinematics[0].jacobian[i].data.row(0);
        const Eigen::RowVectorXd& dp_y_dq = kinematics[0].jacobian[i].data.row(1);
        const Eigen::RowVectorXd& dp_z_dq = kinematics[0].jacobian[i].data.row(2);
        Eigen::MatrixXd& ddp_x_ddq = kinematics[0].hessian[i](0);
        Eigen::MatrixXd& ddp_y_ddq = kinematics[0].hessian[i](1);
        Eigen::MatrixXd& ddp_z_ddq = kinematics[0].hessian[i](2);

        phi(i) = kinematics[0].Phi(i).p.Norm();
        const Eigen::RowVectorXd jacobian_numerator = (kinematics[0].Phi(i).p[0] * kinematics[0].jacobian[i].data.row(0) + kinematics[0].Phi(i).p[1] * kinematics[0].jacobian[i].data.row(1) + kinematics[0].Phi(i).p[2] * kinematics[0].jacobian[i].data.row(2));
        jacobian.row(i).noalias() = jacobian_numerator / phi(i);

        // Two part Hessian
        Eigen::MatrixXd hessian_part1 = -jacobian_numerator.transpose() * jacobian_numerator;
        hessian_part1 /= std::pow((p_x * p_x + p_y * p_y + p_z * p_z), (3 / 2));

        Eigen::MatrixXd hessian_part2 = (p_x * ddp_x_ddq + p_y * ddp_y_ddq + p_z * ddp_z_ddq) + (dp_x_dq.transpose() * dp_x_dq + dp_y_dq.transpose() * dp_y_dq + dp_z_dq.transpose() * dp_z_dq);
        hessian_part2 /= phi(i);

        hessian(i).noalias() = hessian_part1 + hessian_part2;
    }
}

int Distance::TaskSpaceDim()
{
    return kinematics[0].Phi.rows();
}

}  // namespace exotica
