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
    void Instantiate(LevenbergMarquardtSolverInitializer& init) override;

    void Solve(Eigen::MatrixXd& solution) override;

    void SpecifyProblem(PlanningProblemPtr pointer) override;

private:
    LevenbergMarquardtSolverInitializer parameters_;

    UnconstrainedEndPoseProblemPtr prob_;  ///< Shared pointer to the planning problem.

    double lambda_ = 0;  ///< Damping factor
};
}  // namespace exotica

#endif  // EXOTICA_LEVENBERG_MARQUARDT_SOLVER_LEVENBERG_MARQUARDT_SOLVER_H_
