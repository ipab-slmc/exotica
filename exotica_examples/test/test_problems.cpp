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

#include <exotica_core/exotica_core.h>
#include <gtest/gtest.h>

// Extend testing printout //////////////////////

namespace testing
{
namespace internal
{
#if !ROS_VERSION_MINIMUM(1, 15, 0)
enum GTestColor
{
    COLOR_DEFAULT,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW
};
#endif
extern void ColoredPrintf(testing::internal::GTestColor color, const char* fmt, ...);
}  // namespace internal
}  // namespace testing
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

#define CREATE_PROBLEM(X, I) std::shared_ptr<X> problem = CreateProblem<X>(#X, I);
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
    for (int i = 0; i < A.rows(); ++i)
    {
        ret[i] = A[i] - B[i];
    }
    return ret;
}
#endif

template <class T>
std::shared_ptr<T> CreateProblem(const std::string& name, int derivative)
{
    TEST_COUT << "Creating " << name << " with derivatives " << derivative;
    Initializer dummy;
    Initializer init;
    XMLLoader::Load("{exotica_examples}/test/resources/test_problems.xml", dummy, init, "Dummy", name);
    init.AddProperty(Property("DerivativeOrder", false, derivative));
    std::shared_ptr<T> ret = std::static_pointer_cast<T>(Setup::CreateProblem(init));
    TEST_COUT << "Problem loaded";
    return ret;
}

template <class T>
void testJacobianEndPose(std::shared_ptr<T> problem, EndPoseTask& task, double eps = 1e-4, double h = 1e-5)
{
    TEST_COUT << "Testing Jacobian:";
    for (int tr = 0; tr < NUM_TRIALS; ++tr)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0);
        TaskSpaceVector y0 = task.Phi;
        Eigen::MatrixXd J0 = task.jacobian;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; ++i)
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
    for (int tr = 0; tr < NUM_TRIALS; ++tr)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0);

        Hessian H_task = task.hessian;

        Hessian H_fd = Hessian::Constant(problem->length_jacobian, Eigen::MatrixXd::Zero(problem->N, problem->N));
        Eigen::VectorXd x(problem->N);
        for (int j = 0; j < problem->N; ++j)
        {
            x = x0;
            x(j) += h;
            problem->Update(x);
            const Eigen::MatrixXd J1 = problem->jacobian;
            x = x0;
            x(j) -= h;
            problem->Update(x);
            const Eigen::MatrixXd J2 = problem->jacobian;
            for (int i = 0; i < problem->N; ++i)
            {
                for (int k = 0; k < problem->length_jacobian; ++k)
                {
                    H_fd(k)(i, j) = (J1(k, i) - J2(k, i)) / (2.0 * h);
                }
            }
        }
        double errH = 0;
        for (int i = 0; i < H_fd.rows(); ++i) errH += (H_fd(i) - H_task(i)).norm();

        Hessian dH = H_fd - H_task;
        if (errH > eps)
        {
            for (int i = 0; i < dH.rows(); ++i)
            {
                TEST_COUT << "Computed:\n"
                          << H_task(i);
                TEST_COUT << "FD:\n"
                          << H_fd(i);
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
    for (int tr = 0; tr < NUM_TRIALS; ++tr)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0, t);
        TaskSpaceVector y0 = task.Phi[t];
        Eigen::MatrixXd J0 = task.jacobian[t];
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; ++i)
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
    for (int tr = 0; tr < NUM_TRIALS; ++tr)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0, t);

        Hessian H_task = task.hessian[t];

        Hessian H_fd = Hessian::Constant(problem->length_jacobian, Eigen::MatrixXd::Zero(problem->N, problem->N));
        Eigen::VectorXd x(problem->N);
        for (int j = 0; j < problem->N; ++j)
        {
            x = x0;
            x(j) += h;
            problem->Update(x, t);
            const Eigen::MatrixXd J1 = problem->jacobian[t];
            x = x0;
            x(j) -= h;
            problem->Update(x, t);
            const Eigen::MatrixXd J2 = problem->jacobian[t];
            for (int i = 0; i < problem->N; ++i)
            {
                for (int k = 0; k < problem->length_jacobian; ++k)
                {
                    H_fd(k)(i, j) = (J1(k, i) - J2(k, i)) / (2.0 * h);
                }
            }
        }
        double errH = 0;
        for (int i = 0; i < H_fd.rows(); ++i) errH += (H_fd(i) - H_task(i)).norm();

        Hessian dH = H_fd - H_task;
        if (errH > eps)
        {
            for (int i = 0; i < dH.rows(); ++i)
            {
                TEST_COUT << "Computed:\n"
                          << H_task(i);
                TEST_COUT << "FD:\n"
                          << H_fd(i);
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
        for (int d = 0; d < 3; ++d)
        {
            CREATE_PROBLEM(UnconstrainedEndPoseProblem, d);
            Eigen::VectorXd x = problem->GetStartState();
            TEST_COUT << "Testing problem update";
            problem->Update(x);
            TEST_COUT << "Test passed";
            X[d] = problem->cost.ydiff;
            TEST_COUT << "Testing cost";
            if (d > 0)
            {
                J[d] = problem->cost.jacobian;
                testJacobianEndPose(problem, problem->cost);
                if (d > 1)
                {
                    testHessianEndPose(problem, problem->cost);
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            ADD_FAILURE() << "Cost FK is inconsistent!";
        if (!(J[1] == J[2]))
            ADD_FAILURE() << "Cost Jacobians are inconsistent!";
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, BoundedEndPoseProblem)
{
    try
    {
        std::vector<Eigen::VectorXd> X(3);
        std::vector<Eigen::MatrixXd> J(3);
        for (int d = 0; d < 3; ++d)
        {
            CREATE_PROBLEM(BoundedEndPoseProblem, d);
            Eigen::VectorXd x = problem->GetStartState();
            TEST_COUT << "Testing problem update";
            problem->Update(x);
            TEST_COUT << "Test passed";
            X[d] = problem->cost.ydiff;
            TEST_COUT << "Testing cost";
            if (d > 0)
            {
                J[d] = problem->cost.jacobian;
                testJacobianEndPose(problem, problem->cost);
                if (d > 1)
                {
                    testHessianEndPose(problem, problem->cost);
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            ADD_FAILURE() << "Cost FK is inconsistent!";
        if (!(J[1] == J[2]))
            ADD_FAILURE() << "Cost Jacobians are inconsistent!";
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, EndPoseProblem)
{
    try
    {
        std::vector<Eigen::VectorXd> X(9);
        std::vector<Eigen::MatrixXd> J(6);  // Memory layout: {0,1 => Cost, 2,3 => Eq., 4,5 => Neq.}
        for (int d = 0; d < 3; ++d)
        {
            CREATE_PROBLEM(EndPoseProblem, d);
            Eigen::VectorXd x = problem->GetStartState();
            TEST_COUT << "EndPoseProblem: Testing problem update";
            problem->Update(x);
            TEST_COUT << "EndPoseProblem::Update() - Test passed";
            {
                TEST_COUT << "EndPoseProblem: Testing cost";
                X[d] = problem->cost.ydiff;
                if (d > 0)
                {
                    J[d - 1] = problem->cost.jacobian;
                    testJacobianEndPose(problem, problem->cost);
                    if (d > 1)
                    {
                        testHessianEndPose(problem, problem->cost);
                    }
                }
            }
            problem->Update(x);
            {
                TEST_COUT << "EndPoseProblem: Testing equality";
                X[d + 3] = problem->equality.ydiff;
                if (d > 0)
                {
                    J[d + 1] = problem->equality.jacobian;
                    testJacobianEndPose(problem, problem->equality);
                    if (d > 1)
                    {
                        testHessianEndPose(problem, problem->equality);
                    }
                }
            }
            problem->Update(x);
            {
                TEST_COUT << "EndPoseProblem: Testing inequality";
                X[d + 6] = problem->inequality.ydiff;
                if (d > 0)
                {
                    J[d + 3] = problem->inequality.jacobian;
                    testJacobianEndPose(problem, problem->inequality);
                    if (d > 1)
                    {
                        testHessianEndPose(problem, problem->inequality);
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
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, UnconstrainedTimeIndexedProblem)
{
    try
    {
        int T;
        {
            CREATE_PROBLEM(UnconstrainedTimeIndexedProblem, 0);
            T = problem->GetT();
        }
        for (int t = 0; t < T; ++t)
        {
            std::vector<Eigen::VectorXd> X(3);
            std::vector<Eigen::MatrixXd> J(3);
            for (int d = 0; d < 3; ++d)
            {
                CREATE_PROBLEM(UnconstrainedTimeIndexedProblem, d);
                Eigen::VectorXd x = problem->GetStartState();
                TEST_COUT << "Testing problem update";
                problem->Update(x, t);
                TEST_COUT << "Test passed";
                X[d] = problem->cost.ydiff[t];
                TEST_COUT << "Testing cost";
                if (d > 0)
                {
                    J[d] = problem->cost.jacobian[t];
                    testJacobianTimeIndexed(problem, problem->cost, t);
                    if (d > 1)
                    {
                        testHessianTimeIndexed(problem, problem->cost, t);
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                ADD_FAILURE() << "Cost FK is inconsistent!";
            if (!(J[1] == J[2]))
                ADD_FAILURE() << "Cost Jacobians are inconsistent!";
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, BoundedTimeIndexedProblem)
{
    try
    {
        int T;
        {
            CREATE_PROBLEM(BoundedTimeIndexedProblem, 0);
            T = problem->GetT();
        }
        for (int t = 0; t < T; ++t)
        {
            std::vector<Eigen::VectorXd> X(3);
            std::vector<Eigen::MatrixXd> J(3);
            for (int d = 0; d < 3; ++d)
            {
                CREATE_PROBLEM(BoundedTimeIndexedProblem, d);
                Eigen::VectorXd x = problem->GetStartState();
                TEST_COUT << "BoundedTimeIndexedProblem: Testing problem update";
                problem->Update(x, t);
                TEST_COUT << "BoundedTimeIndexedProblem::Update(x): Test passed";
                X[d] = problem->cost.ydiff[t];
                TEST_COUT << "BoundedTimeIndexedProblem: Testing cost";
                if (d > 0)
                {
                    J[d] = problem->cost.jacobian[t];
                    testJacobianTimeIndexed(problem, problem->cost, t);
                    if (d > 1)
                    {
                        testHessianTimeIndexed(problem, problem->cost, t);
                    }
                }
            }
            if (!(X[0].isApprox(X[1]) && X[1].isApprox(X[2])))
                ADD_FAILURE() << "BoundedTimeIndexedProblem: cost value computation is inconsistent!";
            if (!(J[1].isApprox(J[2])))
                ADD_FAILURE() << "BoundedTimeIndexedProblem: cost Jacobians are inconsistent!";
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, TimeIndexedProblem)
{
    try
    {
        int T;
        {
            CREATE_PROBLEM(TimeIndexedProblem, 0);
            T = problem->GetT();
        }
        for (int t = 0; t < T; ++t)
        {
            std::vector<Eigen::VectorXd> X(9);
            std::vector<Eigen::MatrixXd> J(9);
            for (int d = 0; d < 3; ++d)
            {
                CREATE_PROBLEM(TimeIndexedProblem, d);
                Eigen::VectorXd x = problem->GetStartState();
                TEST_COUT << "Testing problem update";
                problem->Update(x, t);
                TEST_COUT << "Test passed";
                {
                    TEST_COUT << "Testing cost";
                    X[d] = problem->cost.ydiff[t];
                    if (d > 0)
                    {
                        J[d] = problem->cost.jacobian[t];
                        testJacobianTimeIndexed(problem, problem->cost, t);
                        if (d > 1)
                        {
                            testHessianTimeIndexed(problem, problem->cost, t);
                        }
                    }
                }
                problem->Update(x, t);
                {
                    TEST_COUT << "Testing equality";
                    X[d + 3] = problem->equality.ydiff[t];
                    if (d > 0)
                    {
                        J[d + 3] = problem->equality.jacobian[t];
                        testJacobianTimeIndexed(problem, problem->equality, t);
                        if (d > 1)
                        {
                            testHessianTimeIndexed(problem, problem->equality, t);
                        }
                    }
                }
                problem->Update(x, t);
                {
                    TEST_COUT << "Testing inequality";
                    X[d + 6] = problem->inequality.ydiff[t];
                    if (d > 0)
                    {
                        J[d + 6] = problem->inequality.jacobian[t];
                        testJacobianTimeIndexed(problem, problem->inequality, t);
                        if (d > 1)
                        {
                            testHessianTimeIndexed(problem, problem->inequality, t);
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
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, SamplingProblem)
{
    try
    {
        CREATE_PROBLEM(SamplingProblem, 0);
        Eigen::VectorXd x = problem->GetStartState();
        TEST_COUT << "Testing problem update";
        problem->Update(x);
        TEST_COUT << "Testing valid state";
        if (!problem->IsValid(x)) ADD_FAILURE() << "Start state is invalid!";
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaProblems, TimeIndexedSamplingProblem)
{
    try
    {
        CREATE_PROBLEM(TimeIndexedSamplingProblem, 0);
        Eigen::VectorXd x = problem->GetStartState();
        TEST_COUT << "Testing problem update";
        problem->Update(x, 0.0);
        TEST_COUT << "Testing valid state";
        if (!problem->IsValid(x, 0.0)) ADD_FAILURE() << "Start state is invalid!";
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    Setup::Destroy();
    return ret;
}
