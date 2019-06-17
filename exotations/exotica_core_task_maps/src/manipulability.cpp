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

#include <exotica_core_task_maps/manipulability.h>

REGISTER_TASKMAP_TYPE("Manipulability", exotica::Manipulability);

namespace exotica
{
void Manipulability::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != TaskSpaceDim()) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < n_end_effs_; ++i)
    {
        const Eigen::MatrixXd& J = kinematics[0].jacobian[i].data.topRows(n_rows_of_jac_);
        phi(i) = lower_bound_(i) - std::sqrt((J * J.transpose()).determinant());
    }
}

void Manipulability::Instantiate(const ManipulabilityInitializer& init)
{
    parameters_ = init;
    n_end_effs_ = frames_.size();

    if (parameters_.LowerBound.rows() == 0)
    {
        lower_bound_.setConstant(n_end_effs_, 1, 0.0);
    }
    else if (parameters_.LowerBound.rows() == 1)
    {
        lower_bound_.setConstant(n_end_effs_, 1, std::abs(static_cast<double>(parameters_.LowerBound(0))));
    }
    else if (parameters_.LowerBound.rows() == n_end_effs_)
    {
        lower_bound_.resize(n_end_effs_, 1);
        lower_bound_ = parameters_.LowerBound.cwiseAbs();
    }
    else
    {
        ThrowNamed("LowerBound vector needs to be either of size 1 or N (N is number of end effectors), but got " << parameters_.LowerBound.rows() << ".");
    }

    if (init.OnlyPosition)
    {
        n_rows_of_jac_ = 3;
    }
    else
    {
        n_rows_of_jac_ = 6;
    }
}

int Manipulability::TaskSpaceDim()
{
    return n_end_effs_;
}
}  // namespace exotica
