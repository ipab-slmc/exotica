#include <exotica/MotionSolver.h>
#include <exotica/Problems/UnconstrainedEndPoseProblem.h>
#include <exotica_levenberg_marquardt_solver/LevenbergMarquardtSolverInitializer.h>

namespace exotica
{
class LevenbergMarquardtSolver : public MotionSolver, public Instantiable<LevenbergMarquardtSolverInitializer>
{
public:
    virtual void Instantiate(LevenbergMarquardtSolverInitializer& init);

    virtual void Solve(Eigen::MatrixXd& solution);

    virtual void specifyProblem(PlanningProblem_ptr pointer);

private:
    LevenbergMarquardtSolverInitializer parameters_;

    UnconstrainedEndPoseProblem_ptr prob_;  // Shared pointer to the planning problem.

    double lambda_ = 0;  // damping factor

    int iterations_ = -1;
};

REGISTER_MOTIONSOLVER_TYPE("LevenbergMarquardtSolverSolver", exotica::LevenbergMarquardtSolver)

void LevenbergMarquardtSolver::Instantiate(LevenbergMarquardtSolverInitializer& init) { parameters_ = init; }
void LevenbergMarquardtSolver::specifyProblem(PlanningProblem_ptr pointer)
{
    if (pointer->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        throw_named("This LevenbergMarquardtSolver can't solve problem of type '" << pointer->type() << "'!");
    }

    MotionSolver::specifyProblem(pointer);

    // generic problem
    problem_ = pointer;

    // specific problem
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(pointer);
}

void LevenbergMarquardtSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->resetCostEvolution(getNumberOfMaxIterations() + 1);

    Timer timer;

    if (!prob_) throw_named("Solver has not been initialized!");

    const Eigen::VectorXd q0 = prob_->applyStartState();

    if (prob_->N != q0.rows()) throw_named("Wrong size q0 size=" << q0.rows() << ", required size=" << prob_->N);

    solution.resize(1, prob_->N);

    lambda_ = parameters_.Damping;  // initial damping

    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(prob_->Cost.J.cols(), prob_->Cost.J.cols());
    Eigen::MatrixXd J;

    Eigen::VectorXd q = q0;
    double error = std::numeric_limits<double>::infinity();
    double error_prev = std::numeric_limits<double>::infinity();
    Eigen::VectorXd yd;
    Eigen::VectorXd qd;
    for (size_t i = 0; i < getNumberOfMaxIterations(); iterations_ = ++i)
    {
        prob_->Update(q);

        yd = prob_->Cost.S * prob_->Cost.ydiff;

        // weighted sum of squares
        error_prev = error;
        error = prob_->getScalarCost();

        prob_->setCostEvolution(i, error);

        J = prob_->Cost.S * prob_->Cost.J;

        // source: https://uk.mathworks.com/help/optim/ug/least-squares-model-fitting-algorithms.html, eq. 13

        if (i > 0)
        {
            if (error < error_prev)
            {
                // success, error decreased: decrease damping
                lambda_ = lambda_ / 10.0;
            }
            else
            {
                // failure, error increased: increase damping
                lambda_ = lambda_ * 10.0;
            }
        }

        if (debug_) HIGHLIGHT_NAMED("Levenberg-Marquardt", "damping: " << lambda_);

        Eigen::MatrixXd M;
        if (parameters_.ScaleProblem == "none")
        {
            M = I;
        }
        else if (parameters_.ScaleProblem == "Jacobian")
        {
            M = (J.transpose() * J).diagonal().asDiagonal();
        }
        else
        {
            throw std::runtime_error("no ScaleProblem of type " + parameters_.ScaleProblem);
        }

        qd = (J.transpose() * J + lambda_ * M).completeOrthogonalDecomposition().solve(J.transpose() * yd);

        if (parameters_.Alpha.size() == 1)
        {
            q -= qd * parameters_.Alpha[0];
        }
        else
        {
            q -= qd.cwiseProduct(parameters_.Alpha);
        }

        if (qd.norm() < parameters_.Convergence)
        {
            if (debug_) HIGHLIGHT_NAMED("Levenberg-Marquardt", "Reached convergence (" << qd.norm() << " < " << parameters_.Convergence << ")");
            break;
        }
    }

    solution.row(0) = q;

    planning_time_ = timer.getDuration();
}

}  // namespace exotica
