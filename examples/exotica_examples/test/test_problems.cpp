#include <exotica/Exotica.h>
#include <gtest/gtest.h>

// Extend testing printout //////////////////////

namespace testing
{
namespace internal
{
enum GTestColor
{
    COLOR_DEFAULT,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW
};

extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
}
}
#define PRINTF(...)                                                                        \
    do                                                                                     \
    {                                                                                      \
        testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); \
        testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__);    \
    } while (0)

// C++ stream interface
class TestCout : public std::stringstream
{
public:
    ~TestCout()
    {
        PRINTF("%s\n", str().c_str());
    }
};

#define TEST_COUT TestCout()

//////////////////////////////////////////////

using namespace exotica;
#include <string>
#include <vector>

#define CREATE_PROBLEM(X, I) std::shared_ptr<X> problem = createProblem<X>(#X, I);
#define NUM_TRIALS 100

// workaround for "error: ambiguous overload for ‘operator-’" that appear in newer Eigen versions
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
Hessian operator-(const Hessian& A, const Hessian& B)
{
    if (A.rows() != B.rows())
    {
        throw std::runtime_error("Hessian dimension mismatch");
    }

    Hessian ret(A.rows());
    for (size_t i = 0; i < A.rows(); i++)
    {
        ret[i] = A[i] - B[i];
    }
    return ret;
}
#endif

template <class T>
std::shared_ptr<T> createProblem(const std::string& name, int derivative)
{
    TEST_COUT << "Creating " << name << " with derivatives " << derivative;
    Initializer dummy;
    Initializer init;
    XMLLoader::load("{exotica_examples}/test/resources/test_problems.xml", dummy, init, "Dummy", name);
    init.addProperty(Property("DerivativeOrder", false, derivative));
    std::shared_ptr<T> ret = std::static_pointer_cast<T>(Setup::createProblem(init));
    TEST_COUT << "Problem loaded";
    return ret;
}

template <class T>
void testJacobianEndPose(std::shared_ptr<T> problem, EndPoseTask& task, double eps = 1e-4, double h = 1e-5)
{
    TEST_COUT << "Testing Jacobian:";
    for (int tr = 0; tr < NUM_TRIALS; tr++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0);
        TaskSpaceVector y0 = task.Phi;
        Eigen::MatrixXd J0 = task.J;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += h;
            problem->Update(x);
            J.col(i) = (task.Phi - y0) / h;
        }
        double errJ = (J - J0).norm();
        if (errJ > eps)
        {
            TEST_COUT << "x: " << x0.transpose();
            TEST_COUT << "J*:\n"
                      << J;
            TEST_COUT << "J:\n"
                      << J0;
            ADD_FAILURE() << "Jacobian error out of bounds: " << errJ;
        }
    }
    TEST_COUT << "Test passed";
}

template <class T>
void testHessianEndPose(std::shared_ptr<T> problem, EndPoseTask& task, double eps = 1e-4, double h = 1e-5)
{
    TEST_COUT << "Testing Hessian:";
    for (int tr = 0; tr < NUM_TRIALS; tr++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0);
        Eigen::MatrixXd J0 = task.J;
        Hessian H0 = task.H;
        Hessian H = H0;
        Hessian H1 = H0;
        for (int k = 0; k < task.JN; k++)
        {
            H1(k) = J0.row(k).transpose() * J0.row(k);
        }
        for (int i = 0; i < H.rows(); i++) H(i).setZero();
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += h;
            problem->Update(x);
            Eigen::MatrixXd Ji = task.J;
            for (int k = 0; k < task.JN; k++)
            {
                H(k).row(i) = (Ji.row(k) - J0.row(k)) / h;
            }
        }
        Hessian dH = H - H0;
        Hessian dH1 = H1 - H0;
        double errH = 0.0;
        for (int i = 0; i < dH.rows(); i++)
            errH = std::min(std::max(errH, dH(i).array().cwiseAbs().maxCoeff()),
                            std::max(errH, dH1(i).array().cwiseAbs().maxCoeff()));
        if (errH > eps)
        {
            for (int i = 0; i < dH.rows(); i++)
            {
                TEST_COUT << "Computed:\n"
                          << H0(i);
                TEST_COUT << "FD:\n"
                          << H(i);
                TEST_COUT << "Diff:\n"
                          << dH(i);
            }
            ADD_FAILURE() << "Hessian error out of bounds: " << errH;
        }
    }
    TEST_COUT << "Test passed";
}

template <class T>
void testJacobianTimeIndexed(std::shared_ptr<T> problem, TimeIndexedTask& task, int t, double eps = 1e-4, double h = 1e-5)
{
    TEST_COUT << "Testing Jacobian:";
    for (int tr = 0; tr < NUM_TRIALS; tr++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0, t);
        TaskSpaceVector y0 = task.Phi[t];
        Eigen::MatrixXd J0 = task.J[t];
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += h;
            problem->Update(x, t);
            J.col(i) = (task.Phi[t] - y0) / h;
        }
        double errJ = (J - J0).norm();
        if (errJ > eps)
        {
            TEST_COUT << "x: " << x0.transpose();
            TEST_COUT << "J*:\n"
                      << J;
            TEST_COUT << "J:\n"
                      << J0;
            ADD_FAILURE() << "Jacobian error out of bounds: " << errJ;
        }
    }
    TEST_COUT << "Test passed";
}

template <class T>
void testHessianTimeIndexed(std::shared_ptr<T> problem, TimeIndexedTask& task, int t, double eps = 1e-4, double h = 1e-5)
{
    TEST_COUT << "Testing Hessian:";
    for (int tr = 0; tr < NUM_TRIALS; tr++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0, t);
        Eigen::MatrixXd J0 = task.J[t];
        Hessian H0 = task.H[t];
        Hessian H = H0;
        Hessian H1 = H0;
        for (int k = 0; k < task.JN; k++)
        {
            H1(k) = J0.row(k).transpose() * J0.row(k);
        }
        for (int i = 0; i < H.rows(); i++) H(i).setZero();
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += h;
            problem->Update(x, t);
            Eigen::MatrixXd Ji = task.J[t];
            for (int k = 0; k < task.JN; k++)
            {
                H(k).row(i) = (Ji.row(k) - J0.row(k)) / h;
            }
        }
        Hessian dH = H - H0;
        Hessian dH1 = H1 - H0;
        double errH = 0.0;
        for (int i = 0; i < dH.rows(); i++)
            errH = std::min(std::max(errH, dH(i).array().cwiseAbs().maxCoeff()),
                            std::max(errH, dH1(i).array().cwiseAbs().maxCoeff()));
        if (errH > eps)
        {
            for (int i = 0; i < dH.rows(); i++)
            {
                TEST_COUT << "Computed:\n"
                          << H0(i);
                TEST_COUT << "FD:\n"
                          << H(i);
                TEST_COUT << "Diff:\n"
                          << dH(i);
            }
            ADD_FAILURE() << "Hessian error out of bounds: " << errH;
        }
    }
    TEST_COUT << "Test passed";
}

TEST(ExoticaProblems, UnconstrainedEndPoseProblem)
{
    try
    {
        std::vector<Eigen::VectorXd> X(3);
        std::vector<Eigen::MatrixXd> J(3);
        for (int d = 0; d < 3; d++)
        {
            CREATE_PROBLEM(UnconstrainedEndPoseProblem, d);
            Eigen::VectorXd x = problem->getStartState();
            TEST_COUT << "Testing problem update";
            problem->Update(x);
            TEST_COUT << "Test passed";
            X[d] = problem->Cost.ydiff;
            TEST_COUT << "Testing cost";
            if (d > 0)
            {
                J[d] = problem->Cost.J;
                testJacobianEndPose(problem, problem->Cost);
                if (d > 1)
                {
                    testHessianEndPose(problem, problem->Cost);
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            ADD_FAILURE() << "Cost FK is inconsistent!";
        if (!(J[1] == J[2]))
            ADD_FAILURE() << "Cost Jacobians are inconsistent!";
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, BoundedEndPoseProblem)
{
    try
    {
        std::vector<Eigen::VectorXd> X(3);
        std::vector<Eigen::MatrixXd> J(3);
        for (int d = 0; d < 3; d++)
        {
            CREATE_PROBLEM(BoundedEndPoseProblem, d);
            Eigen::VectorXd x = problem->getStartState();
            TEST_COUT << "Testing problem update";
            problem->Update(x);
            TEST_COUT << "Test passed";
            X[d] = problem->Cost.ydiff;
            TEST_COUT << "Testing cost";
            if (d > 0)
            {
                J[d] = problem->Cost.J;
                testJacobianEndPose(problem, problem->Cost);
                if (d > 1)
                {
                    testHessianEndPose(problem, problem->Cost);
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            ADD_FAILURE() << "Cost FK is inconsistent!";
        if (!(J[1] == J[2]))
            ADD_FAILURE() << "Cost Jacobians are inconsistent!";
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, EndPoseProblem)
{
    try
    {
        std::vector<Eigen::VectorXd> X(9);
        std::vector<Eigen::MatrixXd> J(6);  // Memory layout: {0,1 => Cost, 2,3 => Eq., 4,5 => Neq.}
        for (int d = 0; d < 3; d++)
        {
            CREATE_PROBLEM(EndPoseProblem, d);
            Eigen::VectorXd x = problem->getStartState();
            TEST_COUT << "EndPoseProblem: Testing problem update";
            problem->Update(x);
            TEST_COUT << "EndPoseProblem::Update() - Test passed";
            {
                TEST_COUT << "EndPoseProblem: Testing cost";
                X[d] = problem->Cost.ydiff;
                if (d > 0)
                {
                    J[d - 1] = problem->Cost.J;
                    testJacobianEndPose(problem, problem->Cost);
                    if (d > 1)
                    {
                        testHessianEndPose(problem, problem->Cost);
                    }
                }
            }
            problem->Update(x);
            {
                TEST_COUT << "EndPoseProblem: Testing equality";
                X[d + 3] = problem->Equality.ydiff;
                if (d > 0)
                {
                    J[d + 1] = problem->Equality.J;
                    testJacobianEndPose(problem, problem->Equality);
                    if (d > 1)
                    {
                        testHessianEndPose(problem, problem->Equality);
                    }
                }
            }
            problem->Update(x);
            {
                TEST_COUT << "EndPoseProblem: Testing inequality";
                X[d + 6] = problem->Inequality.ydiff;
                if (d > 0)
                {
                    J[d + 3] = problem->Inequality.J;
                    testJacobianEndPose(problem, problem->Inequality);
                    if (d > 1)
                    {
                        testHessianEndPose(problem, problem->Inequality);
                    }
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            ADD_FAILURE() << "EndPoseProblem: Cost value computation is inconsistent!";
        if (!(J[0] == J[1]))
            ADD_FAILURE() << "EndPoseProblem: Cost Jacobians are inconsistent!";
        if (!(X[3] == X[4] && X[4] == X[5]))
            ADD_FAILURE() << "EndPoseProblem: Equality value computation is inconsistent!";
        if (!(J[2] == J[3]))
            ADD_FAILURE() << "EndPoseProblem: Equality Jacobians are inconsistent!";
        if (!(X[6] == X[7] && X[7] == X[8]))
            ADD_FAILURE() << "EndPoseProblem: Inequality value computation is inconsistent!";
        if (!(J[4] == J[5]))
            ADD_FAILURE() << "EndPoseProblem: Inequality Jacobians are inconsistent!";
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, UnconstrainedTimeIndexedProblem)
{
    try
    {
        int T;
        {
            CREATE_PROBLEM(UnconstrainedTimeIndexedProblem, 0);
            T = problem->getT();
        }
        for (int t = 0; t < T; t++)
        {
            std::vector<Eigen::VectorXd> X(3);
            std::vector<Eigen::MatrixXd> J(3);
            for (int d = 0; d < 3; d++)
            {
                CREATE_PROBLEM(UnconstrainedTimeIndexedProblem, d);
                Eigen::VectorXd x = problem->getStartState();
                TEST_COUT << "Testing problem update";
                problem->Update(x, t);
                TEST_COUT << "Test passed";
                X[d] = problem->Cost.ydiff[t];
                TEST_COUT << "Testing cost";
                if (d > 0)
                {
                    J[d] = problem->Cost.J[t];
                    testJacobianTimeIndexed(problem, problem->Cost, t);
                    if (d > 1)
                    {
                        testHessianTimeIndexed(problem, problem->Cost, t);
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                ADD_FAILURE() << "Cost FK is inconsistent!";
            if (!(J[1] == J[2]))
                ADD_FAILURE() << "Cost Jacobians are inconsistent!";
        }
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, BoundedTimeIndexedProblem)
{
    try
    {
        int T;
        {
            CREATE_PROBLEM(BoundedTimeIndexedProblem, 0);
            T = problem->getT();
        }
        for (int t = 0; t < T; t++)
        {
            std::vector<Eigen::VectorXd> X(3);
            std::vector<Eigen::MatrixXd> J(3);
            for (int d = 0; d < 3; d++)
            {
                CREATE_PROBLEM(BoundedTimeIndexedProblem, d);
                Eigen::VectorXd x = problem->getStartState();
                TEST_COUT << "Testing problem update";
                problem->Update(x, t);
                TEST_COUT << "Test passed";
                X[d] = problem->Cost.ydiff[t];
                TEST_COUT << "Testing cost";
                if (d > 0)
                {
                    J[d] = problem->Cost.J[t];
                    testJacobianTimeIndexed(problem, problem->Cost, t);
                    if (d > 1)
                    {
                        testHessianTimeIndexed(problem, problem->Cost, t);
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                ADD_FAILURE() << "Cost FK is inconsistent!";
            if (!(J[1] == J[2]))
                ADD_FAILURE() << "Cost Jacobians are inconsistent!";
        }
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, TimeIndexedProblem)
{
    try
    {
        int T;
        {
            CREATE_PROBLEM(TimeIndexedProblem, 0);
            T = problem->getT();
        }
        for (int t = 0; t < T; t++)
        {
            std::vector<Eigen::VectorXd> X(9);
            std::vector<Eigen::MatrixXd> J(9);
            for (int d = 0; d < 3; d++)
            {
                CREATE_PROBLEM(TimeIndexedProblem, d);
                Eigen::VectorXd x = problem->getStartState();
                TEST_COUT << "Testing problem update";
                problem->Update(x, t);
                TEST_COUT << "Test passed";
                {
                    TEST_COUT << "Testing cost";
                    X[d] = problem->Cost.ydiff[t];
                    if (d > 0)
                    {
                        J[d] = problem->Cost.J[t];
                        testJacobianTimeIndexed(problem, problem->Cost, t);
                        if (d > 1)
                        {
                            testHessianTimeIndexed(problem, problem->Cost, t);
                        }
                    }
                }
                problem->Update(x, t);
                {
                    TEST_COUT << "Testing equality";
                    X[d + 3] = problem->Equality.ydiff[t];
                    if (d > 0)
                    {
                        J[d + 3] = problem->Equality.J[t];
                        testJacobianTimeIndexed(problem, problem->Equality, t);
                        if (d > 1)
                        {
                            testHessianTimeIndexed(problem, problem->Equality, t);
                        }
                    }
                }
                problem->Update(x, t);
                {
                    TEST_COUT << "Testing inequality";
                    X[d + 6] = problem->Inequality.ydiff[t];
                    if (d > 0)
                    {
                        J[d + 6] = problem->Inequality.J[t];
                        testJacobianTimeIndexed(problem, problem->Inequality, t);
                        if (d > 1)
                        {
                            testHessianTimeIndexed(problem, problem->Inequality, t);
                        }
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                ADD_FAILURE() << "Cost FK is inconsistent!";
            if (!(J[1] == J[2]))
                ADD_FAILURE() << "Cost Jacobians are inconsistent!";
            if (!(X[3] == X[4] && X[4] == X[5]))
                ADD_FAILURE() << "Equality FK is inconsistent!";
            if (!(J[4] == J[5]))
                ADD_FAILURE() << "Equality Jacobians are inconsistent!";
            if (!(X[6] == X[7] && X[7] == X[8]))
                ADD_FAILURE() << "Inequality FK is inconsistent!";
            if (!(J[7] == J[8]))
                ADD_FAILURE() << "Inequality Jacobians are inconsistent!";
        }
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, SamplingProblem)
{
    try
    {
        CREATE_PROBLEM(SamplingProblem, 0);
        Eigen::VectorXd x = problem->getStartState();
        TEST_COUT << "Testing problem update";
        problem->Update(x);
        TEST_COUT << "Testing valid state";
        if (!problem->isValid(x)) ADD_FAILURE() << "Start state is invalid!";
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

TEST(ExoticaProblems, TimeIndexedSamplingProblem)
{
    try
    {
        CREATE_PROBLEM(TimeIndexedSamplingProblem, 0);
        Eigen::VectorXd x = problem->getStartState();
        TEST_COUT << "Testing problem update";
        problem->Update(x, 0.0);
        TEST_COUT << "Testing valid state";
        if (!problem->isValid(x, 0.0)) ADD_FAILURE() << "Start state is invalid!";
    }
    catch (...)
    {
        ADD_FAILURE() << "Uncaught exception!";
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    Setup::Destroy();
    return ret;
}
