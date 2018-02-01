#include <exotica/Exotica.h>

using namespace exotica;
#include <string>
#include <vector>

#define CREATE_PROBLEM(X, I) std::shared_ptr<X> problem = createProblem<X>(#X, I);
#define NUM_TRIALS 100

template <class T>
std::shared_ptr<T> createProblem(const std::string& name, int derivative)
{
    HIGHLIGHT("Creating " << name << " with derivatives " << derivative);
    Initializer dummy;
    Initializer init;
    XMLLoader::load("{exotica_examples}/resources/configs/test_problems.xml", dummy, init, "Dummy", name);
    init.addProperty(Property("DerivativeOrder", false, derivative));
    std::shared_ptr<T> ret = std::static_pointer_cast<T>(Setup::createProblem(init));
    INFO_PLAIN("Problem loaded");
    return ret;
}

template <class T>
void testJacobianEndPose(std::shared_ptr<T> problem, EndPoseTask& task, double eps = 1e-5)
{
    INFO_PLAIN("Testing Jacobian:");
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
            x(i) += eps;
            problem->Update(x);
            J.col(i) = (task.Phi - y0) / eps;
        }
        double errJ = (J - J0).norm();
        if (errJ > eps)
        {
            HIGHLIGHT("x: " << x0.transpose());
            HIGHLIGHT("J*:\n"
                      << J);
            HIGHLIGHT("J:\n"
                      << J0);
            throw_pretty("Jacobian error out of bounds: " << errJ);
        }
    }
    INFO_PLAIN("Test passed");
}

template <class T>
void testHessianEndPose(std::shared_ptr<T> problem, EndPoseTask& task, double eps = 1e-5)
{
    INFO_PLAIN("Testing Hessian:");
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
            H1(k) = J0.row(k).transpose()*J0.row(k);
        }
        for (int i = 0; i < H.rows(); i++) H(i).setZero();
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += eps;
            problem->Update(x);
            Eigen::MatrixXd Ji = task.J;
            for (int k = 0; k < task.JN; k++)
            {
                H(k).row(i) = (Ji.row(k) - J0.row(k))/eps;
            }
        }
        Hessian dH=H-H0;
        Hessian dH1=H1-H0;
        double errH = 0.0;
        for(int i=0;i<dH.rows();i++)
            errH=std::min(std::max(errH,dH(i).array().cwiseAbs().maxCoeff()),
                          std::max(errH,dH1(i).array().cwiseAbs().maxCoeff()));
        if (errH > eps)
        {
            for(int i=0;i<dH.rows();i++)
            {
                HIGHLIGHT("Computed:\n"<<H0(i));
                HIGHLIGHT("FD:\n"<<H(i));
                HIGHLIGHT("Diff:\n"<<dH(i));
            }
            throw_pretty("Hessian error out of bounds: " << errH);
        }
    }
    INFO_PLAIN("Test passed");
}

template <class T>
void testJacobianTimeIndexed(std::shared_ptr<T> problem, TimeIndexedTask& task, int t, double eps = 1e-5)
{
    INFO_PLAIN("Testing Jacobian:");
    for (int tr = 0; tr < NUM_TRIALS; tr++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0,t);
        TaskSpaceVector y0 = task.Phi[t];
        Eigen::MatrixXd J0 = task.J[t];
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += eps;
            problem->Update(x,t);
            J.col(i) = (task.Phi[t] - y0) / eps;
        }
        double errJ = (J - J0).norm();
        if (errJ > eps)
        {
            HIGHLIGHT("x: " << x0.transpose());
            HIGHLIGHT("J*:\n"
                      << J);
            HIGHLIGHT("J:\n"
                      << J0);
            throw_pretty("Jacobian error out of bounds: " << errJ);
        }
    }
    INFO_PLAIN("Test passed");
}

template <class T>
void testHessianTimeIndexed(std::shared_ptr<T> problem, TimeIndexedTask& task, int t, double eps = 1e-5)
{
    INFO_PLAIN("Testing Hessian:");
    for (int tr = 0; tr < NUM_TRIALS; tr++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0,t);
        Eigen::MatrixXd J0 = task.J[t];
        Hessian H0 = task.H[t];
        Hessian H = H0;
        Hessian H1 = H0;
        for (int k = 0; k < task.JN; k++)
        {
            H1(k) = J0.row(k).transpose()*J0.row(k);
        }
        for (int i = 0; i < H.rows(); i++) H(i).setZero();
        for (int i = 0; i < problem->N; i++)
        {
            Eigen::VectorXd x = x0;
            x(i) += eps;
            problem->Update(x,t);
            Eigen::MatrixXd Ji = task.J[t];
            for (int k = 0; k < task.JN; k++)
            {
                H(k).row(i) = (Ji.row(k) - J0.row(k))/eps;
            }
        }
        Hessian dH=H-H0;
        Hessian dH1=H1-H0;
        double errH = 0.0;
        for(int i=0;i<dH.rows();i++)
            errH=std::min(std::max(errH,dH(i).array().cwiseAbs().maxCoeff()),
                          std::max(errH,dH1(i).array().cwiseAbs().maxCoeff()));
        if (errH > eps)
        {
            for(int i=0;i<dH.rows();i++)
            {
                HIGHLIGHT("Computed:\n"<<H0(i));
                HIGHLIGHT("FD:\n"<<H(i));
                HIGHLIGHT("Diff:\n"<<dH(i));
            }
            throw_pretty("Hessian error out of bounds: " << errH);
        }
    }
    INFO_PLAIN("Test passed");
}

int main(int argc, char** argv)
{
    {
        std::vector<Eigen::VectorXd> X(3);
        std::vector<Eigen::MatrixXd> J(3);
        for (int d = 0; d < 3; d++)
        {
            CREATE_PROBLEM(UnconstrainedEndPoseProblem, d);
            Eigen::VectorXd x = problem->getStartState();
            INFO_PLAIN("Testing problem update");
            problem->Update(x);
            INFO_PLAIN("Test passed");
            X[d] = problem->Cost.ydiff;
            INFO_PLAIN("Testing cost");
            if (d > 0)
            {
                J[d] = problem->Cost.J;
                testJacobianEndPose(problem, problem->Cost);
                if(d>1)
                {
                    testHessianEndPose(problem, problem->Cost);
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            throw_pretty("Cost FK is inconsistent!");
        if (!(J[1] == J[2]))
            throw_pretty("Cost Jacobians are inconsistent!");
    }
    {
        std::vector<Eigen::VectorXd> X(3);
        std::vector<Eigen::MatrixXd> J(3);
        for (int d = 0; d < 3; d++)
        {
            CREATE_PROBLEM(BoundedEndPoseProblem, d);
            Eigen::VectorXd x = problem->getStartState();
            INFO_PLAIN("Testing problem update");
            problem->Update(x);
            INFO_PLAIN("Test passed");
            X[d] = problem->Cost.ydiff;
            INFO_PLAIN("Testing cost");
            if (d > 0)
            {
                J[d] = problem->Cost.J;
                testJacobianEndPose(problem, problem->Cost);
                if(d>1)
                {
                    testHessianEndPose(problem, problem->Cost);
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            throw_pretty("Cost FK is inconsistent!");
        if (!(J[1] == J[2]))
            throw_pretty("Cost Jacobians are inconsistent!");
    }
    {
        std::vector<Eigen::VectorXd> X(9);
        std::vector<Eigen::MatrixXd> J(9);
        for (int d = 0; d < 3; d++)
        {
            CREATE_PROBLEM(EndPoseProblem, d);
            Eigen::VectorXd x = problem->getStartState();
            INFO_PLAIN("Testing problem update");
            problem->Update(x);
            INFO_PLAIN("Test passed");
            {
                INFO_PLAIN("Testing cost");
                X[d] = problem->Cost.ydiff;
                if (d > 0)
                {
                    J[d - 1] = problem->Cost.J;
                    testJacobianEndPose(problem, problem->Cost);
                    if(d>1)
                    {
                        testHessianEndPose(problem, problem->Cost);
                    }
                }
            }
            problem->Update(x);
            {
                INFO_PLAIN("Testing equality");
                X[d+3] = problem->Equality.ydiff;
                if (d > 0)
                {
                    J[d+3] = problem->Equality.J;
                    testJacobianEndPose(problem, problem->Equality);
                    if(d>1)
                    {
                        testHessianEndPose(problem, problem->Equality);
                    }
                }
            }
            problem->Update(x);
            {
                INFO_PLAIN("Testing inequality");
                X[d+6] = problem->Inequality.ydiff;
                if (d > 0)
                {
                    J[d+6] = problem->Inequality.J;
                    testJacobianEndPose(problem, problem->Inequality);
                    if(d>1)
                    {
                        testHessianEndPose(problem, problem->Inequality);
                    }
                }
            }
        }
        if (!(X[0] == X[1] && X[1] == X[2]))
            throw_pretty("Cost FK is inconsistent!");
        if (!(J[1] == J[2]))
            throw_pretty("Cost Jacobians are inconsistent!");
        if (!(X[3] == X[4] && X[4] == X[5]))
            throw_pretty("Equality FK is inconsistent!");
        if (!(J[4] == J[5]))
            throw_pretty("Equality Jacobians are inconsistent!");
        if (!(X[6] == X[7] && X[7] == X[8]))
            throw_pretty("Inequality FK is inconsistent!");
        if (!(J[7] == J[8]))
            throw_pretty("Inequality Jacobians are inconsistent!");
    }






    {
        int T;
        {
            CREATE_PROBLEM(UnconstrainedTimeIndexedProblem, 0);
            T=problem->getT();
        }
        for(int t=0;t<T;t++)
        {
            std::vector<Eigen::VectorXd> X(3);
            std::vector<Eigen::MatrixXd> J(3);
            for (int d = 0; d < 3; d++)
            {
                CREATE_PROBLEM(UnconstrainedTimeIndexedProblem, d);
                Eigen::VectorXd x = problem->getStartState();
                INFO_PLAIN("Testing problem update");
                problem->Update(x,t);
                INFO_PLAIN("Test passed");
                X[d] = problem->Cost.ydiff[t];
                INFO_PLAIN("Testing cost");
                if (d > 0)
                {
                    J[d] = problem->Cost.J[t];
                    testJacobianTimeIndexed(problem, problem->Cost, t);
                    if(d>1)
                    {
                        testHessianTimeIndexed(problem, problem->Cost, t);
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                throw_pretty("Cost FK is inconsistent!");
            if (!(J[1] == J[2]))
                throw_pretty("Cost Jacobians are inconsistent!");
        }
    }

    {
        int T;
        {
            CREATE_PROBLEM(BoundedTimeIndexedProblem, 0);
            T=problem->getT();
        }
        for(int t=0;t<T;t++)
        {
            std::vector<Eigen::VectorXd> X(3);
            std::vector<Eigen::MatrixXd> J(3);
            for (int d = 0; d < 3; d++)
            {
                CREATE_PROBLEM(BoundedTimeIndexedProblem, d);
                Eigen::VectorXd x = problem->getStartState();
                INFO_PLAIN("Testing problem update");
                problem->Update(x,t);
                INFO_PLAIN("Test passed");
                X[d] = problem->Cost.ydiff[t];
                INFO_PLAIN("Testing cost");
                if (d > 0)
                {
                    J[d] = problem->Cost.J[t];
                    testJacobianTimeIndexed(problem, problem->Cost, t);
                    if(d>1)
                    {
                        testHessianTimeIndexed(problem, problem->Cost, t);
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                throw_pretty("Cost FK is inconsistent!");
            if (!(J[1] == J[2]))
                throw_pretty("Cost Jacobians are inconsistent!");
        }
    }

    {
        int T;
        {
            CREATE_PROBLEM(TimeIndexedProblem, 0);
            T=problem->getT();
        }
        for(int t=0;t<T;t++)
        {
            std::vector<Eigen::VectorXd> X(9);
            std::vector<Eigen::MatrixXd> J(9);
            for (int d = 0; d < 3; d++)
            {
                CREATE_PROBLEM(TimeIndexedProblem, d);
                Eigen::VectorXd x = problem->getStartState();
                INFO_PLAIN("Testing problem update");
                problem->Update(x,t);
                INFO_PLAIN("Test passed");
                {
                    INFO_PLAIN("Testing cost");
                    X[d] = problem->Cost.ydiff[t];
                    if (d > 0)
                    {
                        J[d] = problem->Cost.J[t];
                        testJacobianTimeIndexed(problem, problem->Cost, t);
                        if(d>1)
                        {
                            testHessianTimeIndexed(problem, problem->Cost, t);
                        }
                    }
                }
                problem->Update(x,t);
                {
                    INFO_PLAIN("Testing equality");
                    X[d+3] = problem->Equality.ydiff[t];
                    if (d > 0)
                    {
                        J[d+3] = problem->Equality.J[t];
                        testJacobianTimeIndexed(problem, problem->Equality, t);
                        if(d>1)
                        {
                            testHessianTimeIndexed(problem, problem->Equality, t);
                        }
                    }
                }
                problem->Update(x,t);
                {
                    INFO_PLAIN("Testing inequality");
                    X[d+6] = problem->Inequality.ydiff[t];
                    if (d > 0)
                    {
                        J[d+6] = problem->Inequality.J[t];
                        testJacobianTimeIndexed(problem, problem->Inequality, t);
                        if(d>1)
                        {
                            testHessianTimeIndexed(problem, problem->Inequality, t);
                        }
                    }
                }
            }
            if (!(X[0] == X[1] && X[1] == X[2]))
                throw_pretty("Cost FK is inconsistent!");
            if (!(J[1] == J[2]))
                throw_pretty("Cost Jacobians are inconsistent!");
            if (!(X[3] == X[4] && X[4] == X[5]))
                throw_pretty("Equality FK is inconsistent!");
            if (!(J[4] == J[5]))
                throw_pretty("Equality Jacobians are inconsistent!");
            if (!(X[6] == X[7] && X[7] == X[8]))
                throw_pretty("Inequality FK is inconsistent!");
            if (!(J[7] == J[8]))
                throw_pretty("Inequality Jacobians are inconsistent!");
        }
    }

    {
            CREATE_PROBLEM(SamplingProblem, 0);
            Eigen::VectorXd x = problem->getStartState();
            INFO_PLAIN("Testing problem update");
            problem->Update(x);
            INFO_PLAIN("Test passed");
            INFO_PLAIN("Testing valid state");
            if(!problem->isValid(x)) throw_pretty("Start state is invalid!");
            INFO_PLAIN("Test passed");
    }
    {
            CREATE_PROBLEM(TimeIndexedSamplingProblem, 0);
            Eigen::VectorXd x = problem->getStartState();
            INFO_PLAIN("Testing problem update");
            problem->Update(x, 0.0);
            INFO_PLAIN("Test passed");
            INFO_PLAIN("Testing valid state");
            if(!problem->isValid(x, 0.0)) throw_pretty("Start state is invalid!");
            INFO_PLAIN("Test passed");
    }
    Setup::Destroy();
}
