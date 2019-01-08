//#include <unsupported/Eigen/LevenbergMarquardt>  // For DenseFunctor
#include <unsupported/Eigen/NonLinearOptimization>

#include <exotica/MotionSolver.h>
#include <exotica_core/problems/unconstrained_end_pose_problem.h>
#include <exotica_levenberg_marquardt_solver/EigenLevenbergMarquardtSolver_initializer.h>

namespace Eigen
{
template <typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct DenseFunctor
{
    typedef _Scalar Scalar;
    enum
    {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
    typedef ColPivHouseholderQR<JacobianType> QRSolver;
    const int m_inputs, m_values;

    DenseFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    DenseFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}
    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
    //int operator()(const InputType &x, ValueType& fvec) { }
    // should be defined in derived classes

    //int df(const InputType &x, JacobianType& fjac) { }
    // should be defined in derived classes
};
}

namespace exotica
{
struct UnconstrainedEndPoseProblemDenseFunctorWrapper : Eigen::DenseFunctor<double>
{
    // public:
    UnconstrainedEndPoseProblemDenseFunctorWrapper(int inputs, int values) : Eigen::DenseFunctor<double>(inputs, values)
    {
        HIGHLIGHT("Initialising functor with N " << inputs);  // with " << N << ", " << values);
    }

    int operator()(const InputType& x, ValueType& fvec) const
    {
        HIGHLIGHT_NAMED("x:", x.rows() << " x " << x.cols());
        HIGHLIGHT_NAMED("fvec:", fvec.rows() << " x " << fvec.cols());
        problem_->Update(x);
        fvec.setZero();
        fvec(0) = problem_->GetScalarCost();
        HIGHLIGHT_NAMED("f()", x.transpose() << ": " << fvec.transpose());
        // HIGHLIGHT_NAMED("f()", fvec.transpose());
        return 0;
    }

    int df(const InputType& x, JacobianType& fjac) const
    {
        HIGHLIGHT_NAMED("fjac:", fjac.rows() << " x " << fjac.cols());
        problem_->Update(x);
        fjac.setZero();
        fjac.col(0) = problem_->GetScalarJacobian();
        return 0;
    }

    void specify_problem(UnconstrainedEndPoseProblemPtr problem)
    {
        HIGHLIGHT("Setting problem");
        problem_ = problem;
    }
    // protected:
    UnconstrainedEndPoseProblemPtr problem_ = nullptr;
};

class EigenLevenbergMarquardtSolver : public MotionSolver, public Instantiable<EigenLevenbergMarquardtSolverInitializer>
{
public:
    void Instantiate(EigenLevenbergMarquardtSolverInitializer& init) override;

    void Solve(Eigen::MatrixXd& solution) override;

    void SpecifyProblem(PlanningProblemPtr pointer) override;

private:
    EigenLevenbergMarquardtSolverInitializer parameters_;

    UnconstrainedEndPoseProblemPtr prob_;  // Shared pointer to the planning problem.
};

REGISTER_MOTIONSOLVER_TYPE("EigenLevenbergMarquardtSolverSolver", exotica::EigenLevenbergMarquardtSolver)

void EigenLevenbergMarquardtSolver::Instantiate(EigenLevenbergMarquardtSolverInitializer& init) { parameters_ = init; }
void EigenLevenbergMarquardtSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::UnconstrainedEndPoseProblem")
    {
        ThrowNamed("This EigenLevenbergMarquardtSolver can't solve problem of type '" << pointer->type() << "'!");
    }

    MotionSolver::SpecifyProblem(pointer);

    // generic problem
    problem_ = pointer;

    // specific problem
    prob_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(pointer);
}

void EigenLevenbergMarquardtSolver::Solve(Eigen::MatrixXd& solution)
{
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);

    Timer timer;

    if (!prob_) ThrowNamed("Solver has not been initialized!");

    Eigen::VectorXd q0 = prob_->ApplyStartState();

    if (prob_->N != q0.rows()) ThrowNamed("Wrong size q0 size=" << q0.rows() << ", required size=" << prob_->N);

    solution.resize(1, prob_->N);

    UnconstrainedEndPoseProblemDenseFunctorWrapper functor(prob_->N, 1);
    functor.specify_problem(prob_);
    HIGHLIGHT_NAMED("Sizes", functor.inputs() << ", " << functor.values())
    Eigen::HybridNonLinearSolver<UnconstrainedEndPoseProblemDenseFunctorWrapper> lm(functor);
    // lm.setMaxfev(GetNumberOfMaxIterations());
    int info = lm.solve(q0);

    // std::cout<< "Residual fval: "<< lm.fnorm<< std::endl;
    // std::cout<< "Residual gval: "<< lm.gnorm<< std::endl;

    // HIGHLIGHT_NAMED("EigenLevenbergMarquardtSolver", "Info=" << lm.info() << ", " << lm.nfev());

    solution.row(0) = q0.transpose();

    planning_time_ = timer.GetDuration();
}

}  // namespace exotica
