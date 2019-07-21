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

// This code is based on algorithm developed by Marc Toussaint
// M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
// http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
// Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html

/// \mainpage
/// The \ref AICO is a solver within the EXOTica library.

/// \defgroup AICO Approximate Inference Control (AICO) solver
/// @{
/// The AICO solver was designed to solve finite time horizon time discretized (\f$T\f$ number of time steps) motion planning problem.
/// The AICO solver is defined within the EXOTica framework, therefore it makes use of a specification of the exotica planning problem class (\ref UnconstrainedTimeIndexedProblem) and the underlying tools for initialisation and kinematic computations.
/// The inputs of the system are:
///  - The start point \f$x_0\f$
///  - Problem definition (\ref exotica::UnconstrainedTimeIndexedProblem)
/// @}

/// \defgroup math Math functions
/// @{
/// This is a set of math function extending the functionality of Eigen library.
/// @}

/// \file aico_solver.h
/// \brief Approximate Inference Control

#ifndef EXOTICA_AICO_SOLVER_AICO_DYNAMICS_SOLVER_H_
#define EXOTICA_AICO_SOLVER_AICO_DYNAMICS_SOLVER_H_

// #include <iostream>

#include <exotica_core/motion_solver.h>
#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>
// #include <exotica_core/problems/unconstrained_time_indexed_problem.h>

// #include <exotica_aico_solver/incremental_gaussian.h>
#include <exotica_aico_solver/math_operations.h>

#include <exotica_aico_solver/aico_dynamics_solver_initializer.h>

namespace exotica
{
///\brief Solves motion planning problem using Approximate Inference Control method.
///\ingroup AICO
class AICODynamicsSolver : public MotionSolver, public Instantiable<AICODynamicsSolverInitializer>
{
public:
    ///\brief Solves the problem
    ///@param solution Returned solution trajectory as a vector of joint configurations.
    void Solve(Eigen::MatrixXd& solution) override;

    ///\brief Binds the solver to a specific problem which must be pre-initalised
    ///@param pointer Shared pointer to the motion planning problem
    ///@return        Successful if the problem is a valid DynamicTimeIndexedProblem
    void SpecifyProblem(PlanningProblemPtr pointer) override;

    // Eigen::VectorXd GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const override;

private:
    DynamicTimeIndexedShootingProblemPtr prob_;       ///!< Shared pointer to the planning problem.
    DynamicsSolverPtr dynamics_solver_;               ///!< Shared pointer to the dynamics solver.

    // s matrices
    std::vector<Eigen::VectorXd> s_;
    std::vector<Eigen::MatrixXd> Sinv_;
    std::vector<Eigen::MatrixXd> S_;
    std::vector<Eigen::VectorXd> v_;
    std::vector<Eigen::MatrixXd> V_;
    std::vector<Eigen::MatrixXd> Vinv_;
    std::vector<Eigen::MatrixXd> R_;
    std::vector<Eigen::VectorXd> r_;
    std::vector<Eigen::VectorXd> a_;
    std::vector<Eigen::MatrixXd> A_, B_;
    std::vector<Eigen::VectorXd> b_;
    Eigen::MatrixXd Xhat_, Uhat_;
    Eigen::MatrixXd best_X_, best_U_;
    Eigen::VectorXd rhat;
    std::vector<Eigen::VectorXd> qhat;  //!< Point of linearisation

    void Sweep(Eigen::MatrixXd& solution);
    void UpdateProcess(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, int t);
    double UpdateTaskCosts(int t);

    ///\brief Forward simulates the dynamics using the reference trajectory computed in the backwards pass.
    double ForwardPass();
};
}  // namespace exotica

#endif  // EXOTICA_AICO_SOLVER_AICO_DYNAMICS_SOLVER_H_
