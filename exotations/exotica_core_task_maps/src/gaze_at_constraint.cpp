//
// Copyright (c) 2019, University of Edinburgh
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

#include <exotica_core_task_maps/gaze_at_constraint.h>

REGISTER_TASKMAP_TYPE("GazeAtConstraint", exotica::GazeAtConstraint);

namespace exotica
{
void GazeAtConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");

    for (std::size_t i = 0; i < frames_.size(); ++i)
    {
        const int eff_id = 2 * i;
        Eigen::Vector3d p = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        phi(eff_id) = p(0) * p(0) + p(1) * p(1) - tan_theta_squared_(i) * p(2) * p(2);
        phi(eff_id + 1) = -p(2);
    }
}

void GazeAtConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != TaskSpaceDim() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (std::size_t i = 0; i < frames_.size(); ++i)
    {
        const int eff_id = 2 * i;
        Eigen::Vector3d p = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i).p.data);
        phi(eff_id) = p(0) * p(0) + p(1) * p(1) - tan_theta_squared_(i) * p(2) * p(2);
        phi(eff_id + 1) = -p(2);

        for (int j = 0; j < jacobian.cols(); ++j)
        {
            Eigen::Vector3d pd = kinematics[0].jacobian[i].data.topRows<3>().col(j);
            jacobian(eff_id, j) = 2.0 * (p(0) * pd(0) + p(1) * pd(1) - 2.0 * tan_theta_squared_(i) * p(2) * pd(2));
            jacobian(eff_id + 1, j) = -pd(2);
        }
    }
}

void GazeAtConstraint::Instantiate(const GazeAtConstraintInitializer& init)
{
    parameters_ = init;

    if (init.Theta.size() != static_cast<int>(frames_.size())) ThrowPretty("Incorrect size for Theta (expecting " << frames_.size() << ").");

    tan_theta_squared_.resize(frames_.size());
    for (std::size_t i = 0; i < frames_.size(); ++i)
    {
        const double tan_theta = std::tan(init.Theta[i]);
        tan_theta_squared_(i) = tan_theta * tan_theta;
    }
}

int GazeAtConstraint::TaskSpaceDim()
{
    return 2 * frames_.size();
}
}  // namespace exotica
