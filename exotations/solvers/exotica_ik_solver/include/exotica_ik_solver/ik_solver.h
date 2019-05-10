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

#ifndef EXOTICA_IK_SOLVER_IK_SOLVER_H_
#define EXOTICA_IK_SOLVER_IK_SOLVER_H_

#include <exotica_core/motion_solver.h>
#include <exotica_core/problems/unconstrained_end_pose_problem.h>

#include <exotica_ik_solver/ik_solver_initializer.h>

namespace exotica
{
///
/// \brief	IK position solver
/// The solver constructs and solves a linear program of the following form:
/// \f[
///   (SJW)x=y
/// \f]
/// Where S is a diagonal matrix of task scaling factors rho for each task, jacobian is the Jacobian, W is a diagonal matrix
/// of the joint space weights.
///
/// When regularisation term C is used, it gets appended to the Jacobian and distance to the nominal pose is appended
/// to the y vector. For more details see:
/// https://github.com/ipab-slmc/exotica/pull/465#issuecomment-449021817
///
class IKSolver : public MotionSolver, public Instantiable<IKSolverInitializer>
{
public:
    IKSolver();
    virtual ~IKSolver();

    void Solve(Eigen::MatrixXd& solution) override;

    void SpecifyProblem(PlanningProblemPtr pointer) override;

private:
    void ScaleToStepSize(Eigen::VectorXdRef xd);  //!< \brief Scale the state change vector so that the largest dimension is max. step or smaller.

    UnconstrainedEndPoseProblemPtr prob_;  // Shared pointer to the planning problem.

    Eigen::MatrixXd C_;  //!< \brief Regularisation (use values from interval <0, 1))
    Eigen::MatrixXd W_;  //!< \brief Jointspace weighting
};
}

#endif  // EXOTICA_IK_SOLVER_IK_SOLVER_H_
