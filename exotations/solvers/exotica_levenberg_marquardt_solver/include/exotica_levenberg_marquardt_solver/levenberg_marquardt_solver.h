//
// Copyright 2018, University of Edinburgh
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in
//   all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef EXOTICA_LEVENBERG_MARQUARDT_SOLVER_LEVENBERG_MARQUARDT_SOLVER_H_
#define EXOTICA_LEVENBERG_MARQUARDT_SOLVER_LEVENBERG_MARQUARDT_SOLVER_H_

#include <exotica_core/motion_solver.h>
#include <exotica_core/problems/unconstrained_end_pose_problem.h>

#include <exotica_levenberg_marquardt_solver/levenberg_marquardt_solver_initializer.h>

namespace exotica
{
class LevenbergMarquardtSolver : public MotionSolver, public Instantiable<LevenbergMarquardtSolverInitializer>
{
public:
    void Solve(Eigen::MatrixXd& solution) override;

    void SpecifyProblem(PlanningProblemPtr pointer) override;

private:
    UnconstrainedEndPoseProblemPtr prob_;  ///< Shared pointer to the planning problem.

    // Pre-allocation of variables used during optimisation
    double lambda_ = 0;                ///< Damping factor
    Eigen::MatrixXd M_;                ///< Scaling matrix, used for regularisation
    Eigen::MatrixXd JT_times_J_;       ///< Gauss-Newton Hessian approximation (J^T * J)
    Eigen::VectorXd q_;                ///< Joint configuration vector, used during optimisation
    Eigen::VectorXd qd_;               ///< Change in joint configuration, used during optimisation
    Eigen::VectorXd yd_;               ///< Task space difference/error, used during optimisation
    Eigen::MatrixXd cost_jacobian_;    ///< Jacobian, used during optimisation
    double error_;                     ///< Error, used during optimisation
    double error_prev_;                ///< Previous iteration error, used during optimisation
    Eigen::LLT<Eigen::MatrixXd> llt_;  ///< Cholesky decomposition for J^T*J
};
}  // namespace exotica

#endif  // EXOTICA_LEVENBERG_MARQUARDT_SOLVER_LEVENBERG_MARQUARDT_SOLVER_H_
