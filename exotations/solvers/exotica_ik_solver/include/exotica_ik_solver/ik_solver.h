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

#ifndef EXOTICA_IK_SOLVER_IK_SOLVER_H_
#define EXOTICA_IK_SOLVER_IK_SOLVER_H_

#include <exotica_core/motion_solver.h>
#include <exotica_core/problems/unconstrained_end_pose_problem.h>

#include <exotica_ik_solver/ik_solver_initializer.h>

namespace exotica
{
///
/// \brief	Weighted and Regularized Pseudo-Inverse Inverse Kinematics Solver
/// The solver solves a weighted and regularised pseudo-inverse problem. It uses backtracking line-search and adaptive regularization.
///
class IKSolver : public MotionSolver, public Instantiable<IKSolverInitializer>
{
public:
    void Solve(Eigen::MatrixXd& solution) override;
    void SpecifyProblem(PlanningProblemPtr pointer) override;

private:
    UnconstrainedEndPoseProblemPtr prob_;  // Shared pointer to the planning problem.

    Eigen::MatrixXd W_inv_;        ///< Joint-space weighting (inverse)
    Eigen::VectorXd alpha_space_;  ///< Steplengths for backtracking line-search

    // Pre-allocation of variables used during optimisation
    double stop_;                                  ///< Stop criterion: Norm of cost Jacobian
    double step_;                                  ///< Size of step: Sum of squared norm of qd_
    double lambda_ = 0;                            ///< Damping factor
    double steplength_;                            ///< Accepted steplength
    Eigen::VectorXd q_;                            ///< Joint configuration vector, used during optimisation
    Eigen::VectorXd qd_;                           ///< Change in joint configuration, used during optimisation
    Eigen::VectorXd yd_;                           ///< Task space difference/error, used during optimisation
    Eigen::MatrixXd cost_jacobian_;                ///< Jacobian, used during optimisation
    Eigen::MatrixXd J_pseudo_inverse_;             ///< Jacobian pseudo-inverse, used during optimisation
    double error_;                                 ///< Error, used during optimisation
    double error_prev_;                            ///< Error at previous iteration, used during optimisation
    Eigen::LLT<Eigen::MatrixXd> J_decomposition_;  ///< Cholesky decomposition for the weighted pseudo-inverse
    Eigen::MatrixXd J_tmp_;                        ///< Temporary variable for inverse computation

    // Convergence thresholds
    double th_stop_;  ///< Gradient convergence threshold

    // Regularization
    double regmin_ = 1e-9;     //!< Minimum regularization (will not decrease lower)
    double regmax_ = 1e9;      //!< Maximum regularization (to exit by divergence)
    double regfactor_ = 10.;   //!< Factor by which the regularization gets increased/decreased
    double th_stepdec_ = 0.5;  //!< Step-length threshold used to decrease regularization
    double th_stepinc_ = 0.1;  //!< Step-length threshold used to increase regularization

    void IncreaseRegularization()
    {
        lambda_ *= regfactor_;
        if (lambda_ > regmax_)
        {
            lambda_ = regmax_;
        }
    }

    void DecreaseRegularization()
    {
        lambda_ /= regfactor_;
        if (lambda_ < regmin_)
        {
            lambda_ = regmin_;
        }
    }

    void PrintDebug(const int i)
    {
        if (i % 10 == 0 || i == 0)
        {
            std::cout << "iter \t cost \t       stop \t    grad \t  reg \t step\n";
        }

        std::cout << std::setw(4) << i << "  ";
        std::cout << std::scientific << std::setprecision(5) << error_ << "  ";
        std::cout << step_ << "  " << stop_ << "  ";
        std::cout << lambda_ << "  ";
        std::cout << std::fixed << std::setprecision(4) << steplength_ << '\n';
    }

    void ScaleToStepSize(Eigen::VectorXdRef xd);  ///< To scale to maximum step size
};
}  // namespace exotica

#endif  // EXOTICA_IK_SOLVER_IK_SOLVER_H_
