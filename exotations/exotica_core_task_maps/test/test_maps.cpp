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

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <exotica_core/exotica_core.h>
#include <exotica_core/tools/test_helpers.h>

// TODO(#437): Activate once solution for pointer casting/dynamic loading is found.
// #include <exotica_core_task_maps/JointAccelerationBackwardDifference.h>
// #include <exotica_core_task_maps/JointJerkBackwardDifference.h>
// #include <exotica_core_task_maps/JointVelocityBackwardDifference.h>

using namespace exotica;

static const std::string urdf_string_ = "<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual><collision><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></collision></link><link name=\"link1\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual><collision><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></collision></link><link name=\"link2\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual><collision><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></collision></link><link name=\"link3\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual><collision><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></collision></link><link name=\"endeff\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual><collision><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></collision></link><joint name=\"joint1\" type=\"revolute\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /><limit effort=\"200\" velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint2\" type=\"revolute\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\" velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint3\" type=\"revolute\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\" velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
static const std::string srdf_string_ = "<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";

constexpr bool print_debug_information_ = false;
constexpr int num_trials_ = 100;

bool test_random(UnconstrainedEndPoseProblemPtr problem)
{
    Eigen::VectorXd x(3);
    TEST_COUT << "Testing random configurations:";
    for (int i = 0; i < num_trials_; ++i)
    {
        x = problem->GetScene()->GetKinematicTree().GetRandomControlledState();
        problem->Update(x);
        if (print_debug_information_)
        {
            TEST_COUT << "x = " << x.transpose();
            TEST_COUT << "y = " << problem->Phi.data.transpose();
            TEST_COUT << "jacobian = \n"
                      << problem->jacobian;
        }
    }
    return true;
}

bool test_random(UnconstrainedTimeIndexedProblemPtr problem)
{
    TEST_COUT << "Testing random configurations:";
    for (int i = 0; i < num_trials_; ++i)
    {
        for (int t = 0; t < problem->GetT(); ++t)
        {
            problem->Update(problem->GetScene()->GetKinematicTree().GetRandomControlledState(), t);
        }
    }
    return true;
}

bool test_values(Eigen::MatrixXdRefConst Xref, Eigen::MatrixXdRefConst Yref, Eigen::MatrixXdRefConst Jref, UnconstrainedEndPoseProblemPtr problem, const double eps = 1e-5)
{
    TEST_COUT << "Testing set points:";
    int M = Yref.cols();
    int L = Xref.rows();
    for (int i = 0; i < L; ++i)
    {
        Eigen::VectorXd x = Xref.row(i);
        TaskSpaceVector y = problem->cost.y;
        y.data = Yref.row(i);
        Eigen::MatrixXd jacobian = Jref.middleRows(i * M, M);
        problem->Update(x);
        double errY = (y - problem->Phi).norm();
        double errJ = (jacobian - problem->jacobian).norm();
        if (errY > eps)
        {
            TEST_COUT << "y:  " << problem->Phi.data.transpose();
            TEST_COUT << "y*: " << y.data.transpose();
            ADD_FAILURE() << "Task space error out of bounds: " << errY;
        }
        if (errJ > eps)
        {
            TEST_COUT << "x: " << x.transpose();
            TEST_COUT << "J*:\n"
                      << jacobian;
            TEST_COUT << "J:\n"
                      << problem->jacobian;
            ADD_FAILURE() << "Jacobian error out of bounds: " << errJ;
        }
    }

    return true;
}

bool test_jacobian(UnconstrainedEndPoseProblemPtr problem, const double eps = 1e-5)
{
    constexpr double h = 1e-5;  // NB: Not, this differs from the h for the time-indexed Jacobian

    TEST_COUT << "Testing Jacobian with h=" << h << ", eps=" << eps;
    for (int j = 0; j < num_trials_; ++j)
    {
        Eigen::VectorXd x0(problem->N);
        x0 = problem->GetScene()->GetKinematicTree().GetRandomControlledState();
        problem->Update(x0);
        Eigen::VectorXd x(x0);
        const Eigen::MatrixXd J0(problem->jacobian);
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; ++i)
        {
            x = x0;
            x(i) += h;
            problem->Update(x);
            TaskSpaceVector x_plus(problem->Phi);
            x = x0;
            x(i) -= h;
            problem->Update(x);
            TaskSpaceVector x_minus(problem->Phi);
            jacobian.col(i) = (x_plus - x_minus) / (2.0 * h);
        }
        double errJ = (jacobian - J0).lpNorm<Eigen::Infinity>();
        if (errJ > eps)
        {
            TEST_COUT << "x: " << x0.transpose();
            TEST_COUT << "J*:\n"
                      << jacobian;
            TEST_COUT << "J:\n"
                      << J0;
            TEST_COUT << "(J*-J):\n"
                      << (jacobian - J0);
            ADD_FAILURE() << "Jacobian error out of bounds: " << errJ;
        }
    }
    return true;
}

bool test_hessian(UnconstrainedEndPoseProblemPtr problem, const double eps = 1e-5)
{
    constexpr double h = 1e-5;

    TEST_COUT << "Testing Hessian with h=" << h << ", eps=" << eps;
    for (int l = 0; l < num_trials_; ++l)
    {
        Eigen::VectorXd x0 = problem->GetScene()->GetKinematicTree().GetRandomControlledState();
        problem->Update(x0);
        const Hessian H0(problem->hessian);
        Hessian hessian = Hessian::Constant(problem->length_jacobian, Eigen::MatrixXd::Zero(problem->N, problem->N));
        Eigen::VectorXd x;
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
                    hessian(k)(i, j) = (J1(k, i) - J2(k, i)) / (2.0 * h);
                }
            }
        }
        double errH = 0;
        for (int i = 0; i < hessian.rows(); ++i) errH += (hessian(i) - H0(i)).norm();
        if (errH > eps)
        {
            TEST_COUT << "x: " << x0.transpose();
            TEST_COUT << "H*:\n";
            for (int i = 0; i < problem->length_jacobian; ++i) TEST_COUT << hessian(i) << "\n\n";
            TEST_COUT << "\n\n...";
            TEST_COUT << "H:\n";
            for (int i = 0; i < problem->length_jacobian; ++i) TEST_COUT << H0(i) << "\n\n";
            TEST_COUT << "\n\n...";
            ADD_FAILURE() << "Hessian error out of bounds: " << errH;
        }
    }
    return true;
}

template <class T>
bool test_jacobian_time_indexed(std::shared_ptr<T> problem, TimeIndexedTask& task, int t, const double eps = 1e-5)
{
    constexpr double h = 1e-6;

    TEST_COUT << "Testing Jacobian with h=" << h << ", eps=" << eps;
    for (int tr = 0; tr < num_trials_; ++tr)
    {
        Eigen::VectorXd x0(problem->N);
        x0 = problem->GetScene()->GetKinematicTree().GetRandomControlledState();
        problem->Update(x0, t);
        TaskSpaceVector y0 = task.Phi[t];
        Eigen::MatrixXd J0 = task.jacobian[t];
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for (int i = 0; i < problem->N; ++i)
        {
            Eigen::VectorXd x = x0;
            x(i) += h;
            problem->Update(x, t);
            jacobian.col(i) = (task.Phi[t] - y0) / h;
        }
        double errJ = (jacobian - J0).norm();
        if (errJ > eps)
        {
            TEST_COUT << "x: " << x0.transpose();
            TEST_COUT << "Phi: " << task.Phi[t].data.transpose();
            TEST_COUT << "J*:\n"
                      << jacobian;
            TEST_COUT << "J:\n"
                      << J0;
            ADD_FAILURE() << "Jacobian error out of bounds: " << errJ;
        }
    }
    return true;
}

UnconstrainedEndPoseProblemPtr setup_problem(Initializer& map, const std::string& collision_scene = "", const std::vector<Initializer>& links = std::vector<Initializer>())
{
    Initializer scene;
    if (!collision_scene.empty())
        scene = Initializer("Scene", {{"Name", std::string("MyScene")},
                                      {"JointGroup", std::string("arm")},
                                      {"Links", links},
                                      {"AlwaysUpdateCollisionScene", std::string("1")},
                                      {"CollisionScene", std::vector<Initializer>({Initializer(collision_scene, {{"Name", std::string("MyCollisionScene")}})})}});
    else
        scene = Initializer("Scene", {{"Name", std::string("MyScene")}, {"JointGroup", std::string("arm")}, {"Links", links}});
    Initializer cost("exotica/Task", {{"Task", std::string("MyTask")}});
    Eigen::VectorXd W = Eigen::Vector3d(3, 2, 1);
    Initializer problem("exotica/UnconstrainedEndPoseProblem", {
                                                                   {"Name", std::string("MyProblem")},
                                                                   {"DerivativeOrder", std::string("2")},
                                                                   {"PlanningScene", scene},
                                                                   {"Maps", std::vector<Initializer>({map})},
                                                                   {"Cost", std::vector<Initializer>({cost})},
                                                                   {"W", W},
                                                               });
    Server::Instance()->GetModel("robot_description", urdf_string_, srdf_string_);

    UnconstrainedEndPoseProblemPtr problem_ptr = std::static_pointer_cast<UnconstrainedEndPoseProblem>(Setup::CreateProblem(problem));

    // Create and test a problem with multiple cost terms
    problem = Initializer("exotica/UnconstrainedEndPoseProblem", {
                                                                     {"Name", std::string("MyProblem")},
                                                                     {"PlanningScene", scene},
                                                                     {"Maps", std::vector<Initializer>({map})},
                                                                     {"Cost", std::vector<Initializer>({cost, cost, cost})},
                                                                     {"W", W},
                                                                 });

    test_random(std::static_pointer_cast<UnconstrainedEndPoseProblem>(Setup::CreateProblem(problem)));

    return problem_ptr;
}

UnconstrainedTimeIndexedProblemPtr setup_time_indexed_problem(Initializer& map)
{
    Initializer scene("Scene", {{"Name", std::string("MyScene")}, {"JointGroup", std::string("arm")}});
    Initializer cost("exotica/Task", {{"Task", std::string("MyTask")}});
    Eigen::VectorXd W = Eigen::Vector3d(3, 2, 1);

    Initializer problem("exotica/UnconstrainedTimeIndexedProblem", {{"Name", std::string("MyProblem")},
                                                                    {"PlanningScene", scene},
                                                                    {"Maps", std::vector<Initializer>({map})},
                                                                    {"Cost", std::vector<Initializer>({cost})},
                                                                    {"W", W},
                                                                    {"T", std::string("10")},
                                                                    {"tau", std::string("0.05")}});
    Server::Instance()->GetModel("robot_description", urdf_string_, srdf_string_);

    UnconstrainedTimeIndexedProblemPtr problem_ptr = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(Setup::CreateProblem(problem));

    // Create and test a problem with multiple cost terms
    problem = Initializer("exotica/UnconstrainedTimeIndexedProblem", {{"Name", std::string("MyProblem")},
                                                                      {"PlanningScene", scene},
                                                                      {"Maps", std::vector<Initializer>({map})},
                                                                      {"Cost", std::vector<Initializer>({cost, cost, cost})},
                                                                      {"W", W},
                                                                      {"T", std::string("10")},
                                                                      {"tau", std::string("0.05")}});

    test_random(std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(Setup::CreateProblem(problem)));

    return problem_ptr;
}

TEST(ExoticaTaskMaps, testEffPositionXY)
{
    try
    {
        TEST_COUT << "End-effector position XY test";
        Initializer map("exotica/EffPositionXY", {{"Name", std::string("MyTask")},
                                                  {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));
        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testEffPosition)
{
    try
    {
        TEST_COUT << "End-effector position test";
        Initializer map("exotica/EffPosition", {{"Name", std::string("MyTask")},
                                                {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        int N = problem->N;
        int M = problem->length_Phi;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd jacobian(L * M, N);

        X << 0.680375, -0.211234, 0.566198, 0.59688, 0.823295, -0.604897, -0.329554, 0.536459, -0.444451, 0.10794, -0.0452059, 0.257742, -0.270431, 0.0268018, 0.904459;
        Y << 0.0645323, 0.0522249, 1.21417, 0.292945, 0.199075, 1.12724, 0.208378, -0.0712708, 1.19893, 0.0786457, 0.00852213, 1.23952, 0.356984, -0.0989639, 1.06844;
        jacobian << -0.0522249, 0.594015, 0.327994, 0.0645323, 0.480726, 0.26544, 0, -0.0830172, -0.156401, -0.199075, 0.560144, 0.363351, 0.292945, 0.380655, 0.246921, 0, -0.354186, -0.0974994, 0.0712708, 0.708627, 0.423983, 0.208378, -0.24237, -0.145014, 0, -0.220229, -0.0413455, -0.00852213, 0.784922, 0.437315, 0.0786457, 0.085055, 0.0473879, 0, -0.0791061, -0.0949228, 0.0989639, 0.595968, 0.258809, 0.356984, -0.165215, -0.0717477, 0, -0.370448, -0.361068;
        EXPECT_TRUE(test_values(X, Y, jacobian, problem));

        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testEffOrientation)
{
    try
    {
        TEST_COUT << "End-effector orientation test";
        std::vector<std::string> types = {"Quaternion", "ZYX", "ZYZ", "AngleAxis", "Matrix", "RPY"};

        for (std::size_t i = 0; i < types.size(); ++i)
        {
            std::string type = types[i];
            TEST_COUT << "Rotation type " << type;
            Initializer map("exotica/EffOrientation", {{"Name", std::string("MyTask")},
                                                       {"Type", type},
                                                       {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);

            EXPECT_TRUE(test_random(problem));

            EXPECT_TRUE(test_jacobian(problem));
            EXPECT_TRUE(test_hessian(problem));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testEffAxisAlignment)
{
    try
    {
        TEST_COUT << "End-effector axis alignment test";

        Initializer map("exotica/EffAxisAlignment", {{"Name", std::string("MyTask")},
                                                     {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}, {"Axis", std::string("1 0 0")}, {"Direction", std::string("0 0 1")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        EXPECT_TRUE(test_jacobian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testEffFrame)
{
    try
    {
        TEST_COUT << "End-effector frame test";
        std::vector<std::string> types = {"Quaternion", "ZYX", "ZYZ", "AngleAxis", "Matrix", "RPY"};

        for (std::size_t i = 0; i < types.size(); ++i)
        {
            const std::string& type = types[i];
            TEST_COUT << "Rotation type " << type;
            Initializer map("exotica/EffFrame", {{"Name", std::string("MyTask")},
                                                 {"Type", type},
                                                 {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));

            EXPECT_TRUE(test_jacobian(problem));
            EXPECT_TRUE(test_hessian(problem));
        }

        // Multi-end-effector test case
        for (std::size_t i = 0; i < types.size(); ++i)
        {
            const std::string& type = types[i];
            TEST_COUT << "[Multi end-effector] Rotation type " << type;
            Initializer map("exotica/EffFrame", {{"Name", std::string("MyTask")},
                                                 {"Type", type},
                                                 {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}}),
                                                                                           Initializer("Frame", {{"Link", std::string("link3")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));

            EXPECT_TRUE(test_jacobian(problem));
            EXPECT_TRUE(test_hessian(problem));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testEffVelocity)
{
    try
    {
        TEST_COUT << "End-effector velocity test";

        Initializer map("exotica/EffVelocity", {{"Name", std::string("MyTask")},
                                                {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        UnconstrainedTimeIndexedProblemPtr problem = setup_time_indexed_problem(map);
        EXPECT_TRUE(test_random(problem));

        for (int t = 0; t < problem->GetT(); ++t)
        {
            problem->Update(problem->GetScene()->GetKinematicTree().GetRandomControlledState(), t);
        }

        for (int t = 0; t < problem->GetT(); ++t)
        {
            // TODO: Can we get those tolerances to be tighter?
            EXPECT_TRUE(test_jacobian_time_indexed(problem, problem->cost, t, 1e-3));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testDistance)
{
    try
    {
        TEST_COUT << "Distance test";
        Initializer map("exotica/Distance", {{"Name", std::string("MyTask")},
                                             {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        int N = problem->N;
        int M = problem->length_Phi;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd jacobian(L * M, N);

        X << 0.83239, 0.271423, 0.434594, -0.716795, 0.213938, -0.967399, -0.514226, -0.725537, 0.608354, -0.686642, -0.198111, -0.740419, -0.782382, 0.997849, -0.563486;
        Y << 1.19368, 1.14431, 1.19326, 1.14377, 1.1541;
        jacobian << 0, -0.145441, -0.16562,
            0, 0.0918504, 0.234405,
            0, 0.107422, -0.0555943,
            0, 0.169924, 0.235715,
            0, -0.188516, -0.000946239;
        EXPECT_TRUE(test_values(X, Y, jacobian, problem));

        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem, 1e-2));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testDistanceToLine2D)
{
    try
    {
        // Fixed frames build line
        {
            TEST_COUT << "DistanceToLine2D test: Fixed frames";
            Initializer map("exotica/DistanceToLine2D", {{"Name", std::string("MyTask")},
                                                         {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("base")}, {"BaseOffset", std::string("-0.5 -0.5 0")}}),
                                                                                                   Initializer("Frame", {{"Link", std::string("base")}, {"BaseOffset", std::string("0.5 0.5 0")}}),
                                                                                                   Initializer("Frame", {{"Link", std::string("endeff")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }

        // Robot (moving) frames build line
        {
            TEST_COUT << "DistanceToLine2D test: Moving links";
            Initializer map("exotica/DistanceToLine2D", {{"Name", std::string("MyTask")},
                                                         {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("link1")}}),
                                                                                                   Initializer("Frame", {{"Link", std::string("link3")}}),
                                                                                                   Initializer("Frame", {{"Link", std::string("endeff")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testJointLimit)
{
    try
    {
        TEST_COUT << "Joint limit test";
        Initializer map("exotica/JointLimit", {{"Name", std::string("MyTask")},
                                               {"SafePercentage", 0.0}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));
        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testJointTorqueMinimizationProxy)
{
    try
    {
        TEST_COUT << "Joint torque minimization proxy test";

        Initializer map("exotica/JointTorqueMinimizationProxy", {{"Name", std::string("MyTask")},
                                                                 {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        EXPECT_TRUE(test_jacobian(problem, 1.e-4));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testJointVelocityLimit)
{
    try
    {
        TEST_COUT << "JointVelocityLimit test";

        std::vector<Initializer> maps;
        maps.reserve(3);

        // Test default
        {
            Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(1);
            Initializer map("exotica/JointVelocityLimit", {{"Name", std::string("MyTask")},
                                                           {"dt", 0.1},
                                                           {"MaximumJointVelocity", qd_max},
                                                           {"SafePercentage", 0.0}});
            maps.emplace_back(map);
        }

        // Test safe percentage
        {
            Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(1);
            Initializer map("exotica/JointVelocityLimit", {{"Name", std::string("MyTask")},
                                                           {"dt", 0.1},
                                                           {"MaximumJointVelocity", qd_max},
                                                           {"SafePercentage", 0.1}});
            maps.emplace_back(map);
        }

        // Test different joint velocity initialisations (vector 1, vector N)
        {
            UnconstrainedTimeIndexedProblemPtr problem = setup_time_indexed_problem(maps[0]);
            Eigen::VectorXd qd_max = Eigen::VectorXd::Ones(problem->N);
            Initializer map("exotica/JointVelocityLimit", {{"Name", std::string("MyTask")},
                                                           {"dt", 0.1},
                                                           {"MaximumJointVelocity", qd_max},
                                                           {"SafePercentage", 0.0}});
            maps.emplace_back(map);
        }

        for (auto map : maps)
        {
            UnconstrainedTimeIndexedProblemPtr problem = setup_time_indexed_problem(map);
            EXPECT_TRUE(test_random(problem));

            for (int t = 0; t < problem->GetT(); ++t)
            {
                problem->Update(problem->GetScene()->GetKinematicTree().GetRandomControlledState(), t);
            }

            for (int t = 0; t < problem->GetT(); ++t)
            {
                EXPECT_TRUE(test_jacobian_time_indexed(problem, problem->cost, t, 1e-4));
            }
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "JointVelocityLimit: Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testSphereCollision)
{
    try
    {
        TEST_COUT << "Sphere collision test";
        Initializer map("exotica/SphereCollision", {{"Name", std::string("MyTask")},
                                                    {"Precision", 1e-2},
                                                    {"ReferenceFrame", std::string("base")},
                                                    {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("base")}, {"Radius", 0.3}, {"Group", std::string("base")}}),
                                                                                              Initializer("Frame", {{"Link", std::string("endeff")}, {"Radius", 0.3}, {"Group", std::string("eff")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        int N = problem->N;
        int M = problem->length_Phi;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd jacobian(L * M, N);

        X << -0.590167, 1.2309, 1.67611, -1.72098, 1.79731, 0.103981, -1.65578, -1.23114, 0.652908, 1.56093, -0.604428, -1.74331, -1.91991, -0.169193, -1.74762;
        Y << 1, 5.71023e-44, 1.83279e-110, 6.87352e-16, 7.26371e-45;
        jacobian << 0, 0.431392, 0.449344,
            0, 0.431735, 0.26014,
            0, -0.234475, -0.0135658,
            0, -0.349195, -0.447214,
            0, -0.270171, -0.430172;
        EXPECT_TRUE(test_values(X, Y, jacobian, problem));

        // EXPECT_TRUE(test_jacobian(problem));  // Throws!
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testJointPose)
{
    try
    {
        TEST_COUT << "JointPose test";
        Initializer map("exotica/JointPose", {{"Name", std::string("MyTask")}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << -0.52344, 0.941268, 0.804416, 0.70184, -0.466669, 0.0795207, -0.249586, 0.520497, 0.0250707, 0.335448, 0.0632129, -0.921439, -0.124725, 0.86367, 0.86162;
            Y << -0.52344, 0.941268, 0.804416, 0.70184, -0.466669, 0.0795207, -0.249586, 0.520497, 0.0250707, 0.335448, 0.0632129, -0.921439, -0.124725, 0.86367, 0.86162;
            jacobian << 1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem));

        TEST_COUT << "JointPose test with reference";
        map = Initializer("exotica/JointPose", {{"Name", std::string("MyTask")},
                                                {"JointRef", std::string("0.5 0.5 0.5")}});
        problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << 0.441905, -0.431413, 0.477069, 0.279958, -0.291903, 0.375723, -0.668052, -0.119791, 0.76015, 0.658402, -0.339326, -0.542064, 0.786745, -0.29928, 0.37334;
            Y << -0.0580953, -0.931413, -0.0229314, -0.220042, -0.791903, -0.124277, -1.16805, -0.619791, 0.26015, 0.158402, -0.839326, -1.04206, 0.286745, -0.79928, -0.12666;
            jacobian << 1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem));

        TEST_COUT << "JointPose test with subset of joints";
        map = Initializer("exotica/JointPose", {{"Name", std::string("MyTask")},
                                                {"JointMap", std::string("0")}});
        problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << 0.912937, 0.17728, 0.314608, 0.717353, -0.12088, 0.84794, -0.203127, 0.629534, 0.368437, 0.821944, -0.0350187, -0.56835, 0.900505, 0.840257, -0.70468;
            Y << 0.912937, 0.717353, -0.203127, 0.821944, 0.900505;
            jacobian << 1, 0, 0,
                1, 0, 0,
                1, 0, 0,
                1, 0, 0,
                1, 0, 0;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "JointPose: Uncaught exception!" << e.what();
    }
}

TEST(ExoticaTaskMaps, testCoM)
{
    try
    {
        TEST_COUT << "CoM test";
        Initializer map("exotica/CenterOfMass", {{"Name", std::string("MyTask")},
                                                 {"EnableZ", true}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << -0.281809, 0.10497, 0.15886, -0.0948483, 0.374775, -0.80072, 0.061616, 0.514588, -0.39141, 0.984457, 0.153942, 0.755228, 0.495619, 0.25782, -0.929158;
            Y << 0.081112, -0.0234831, 0.924368, 0.0080578, -0.000766568, 0.895465, 0.157569, 0.00972105, 0.897157, 0.117213, 0.176455, 0.846633, -0.0587457, -0.0317596, 0.877501;
            jacobian << 0.0234831, 0.455657, 0.200919,
                0.081112, -0.131919, -0.0581688,
                0, -0.0844429, -0.0565023,
                0.000766568, 0.443462, 0.19642,
                0.0080578, -0.0421882, -0.0186862,
                0, -0.00809418, 0.0895227,
                -0.00972105, 0.446309, 0.214617,
                0.157569, 0.0275346, 0.0132406,
                0, -0.157868, -0.0266211,
                -0.176455, 0.219463, 0.0736575,
                0.117213, 0.330384, 0.110885,
                0, -0.211838, -0.170949,
                0.0317596, 0.376061, 0.149235,
                -0.0587457, 0.203309, 0.0806805,
                0, 0.0667812, 0.134774;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));

        TEST_COUT << "CoM test with a subset of links";
        map = Initializer("exotica/CenterOfMass", {{"Name", std::string("MyTask")},
                                                   {"EnableZ", true},
                                                   {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("link2")}}),
                                                                                             Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << 0.299414, -0.503912, 0.258959, -0.541726, 0.40124, -0.366266, -0.342446, -0.537144, -0.851678, 0.266144, -0.552687, 0.302264, 0.0213719, 0.942931, -0.439916;
            Y << -0.167532, -0.0517162, 0.913823, 0.083533, -0.0502684, 0.931962, -0.3632, 0.129477, 0.693081, -0.17971, -0.048991, 0.907923, 0.314586, 0.00672432, 0.823106;
            jacobian << 0.0517162, 0.443188, 0.254921,
                -0.167532, 0.13681, 0.0786927,
                0, 0.175333, 0.0666905,
                0.0502684, 0.412954, 0.235481,
                0.083533, -0.248507, -0.141708,
                0, -0.0974919, -0.00961589,
                -0.129477, 0.228967, 0.0468775,
                -0.3632, -0.0816248, -0.0167114,
                0, 0.385588, 0.270459,
                0.048991, 0.441801, 0.257042,
                -0.17971, 0.12044, 0.0700725,
                0, 0.186268, 0.0681488,
                -0.00672432, 0.373021, 0.240882,
                0.314586, 0.00797337, 0.00514888,
                0, -0.314658, -0.132569;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));

        TEST_COUT << "CoM test with projection on XY plane";
        map = Initializer("exotica/CenterOfMass", {{"Name", std::string("MyTask")},
                                                   {"EnableZ", false}});
        problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << 0.350952, -0.0340994, -0.0361284, -0.390089, 0.424175, -0.634888, 0.243646, -0.918271, -0.172033, 0.391968, 0.347873, 0.27528, -0.305768, -0.630755, 0.218212;
            Y << -0.0228141, -0.0083524, 0.0595937, -0.0245025, -0.392081, -0.0974653, 0.200868, 0.0830305, -0.232814, 0.0734919;
            jacobian << 0.0083524, 0.453225, 0.202958,
                -0.0228141, 0.165929, 0.0743046,
                0.0245025, 0.420734, 0.195957,
                0.0595937, -0.172988, -0.0805696,
                0.0974653, 0.254325, 0.0971889,
                -0.392081, 0.0632213, 0.0241597,
                -0.0830305, 0.394279, 0.162599,
                0.200868, 0.162978, 0.0672114,
                -0.0734919, 0.394649, 0.189283,
                -0.232814, -0.124578, -0.0597504;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));

        TEST_COUT << "CoM test with attached object";
        map = Initializer("exotica/CenterOfMass", {{"Name", std::string("MyTask")},
                                                   {"EnableZ", true}});
        problem = setup_problem(map);

        problem->GetScene()->AddObject("Payload", KDL::Frame(), "", shapes::ShapeConstPtr(nullptr), KDL::RigidBodyInertia(0.5));
        problem->GetScene()->AttachObjectLocal("Payload", "endeff", KDL::Frame());
        EXPECT_TRUE(test_random(problem));
        {
            int N = problem->N;
            int M = problem->length_Phi;
            int L = 5;
            Eigen::MatrixXd X(L, N);
            Eigen::MatrixXd Y(L, M);
            Eigen::MatrixXd jacobian(L * M, N);

            X << 0.792099, -0.891848, -0.781543, 0.877611, 0.29783, 0.452939, 0.988809, 0.86931, 0.270667, -0.201327, -0.925895, 0.0373103, 0.0433417, 0.560965, -0.682102;
            Y << -0.391939, -0.397228, 0.608195, 0.197788, 0.238097, 0.977105, 0.289066, 0.439301, 0.781316, -0.4839, 0.0987601, 0.836551, 0.122905, 0.00533025, 1.02823;
            jacobian << 0.397228, 0.111109, -0.0232142,
                -0.391939, 0.112608, -0.0235274,
                0, 0.558038, 0.32103,
                -0.238097, 0.336815, 0.150781,
                0.197788, 0.405457, 0.181509,
                0, -0.309533, -0.220165,
                -0.439301, 0.182119, 0.0740843,
                0.289066, 0.276772, 0.112588,
                0, -0.525875, -0.293238,
                -0.0987601, 0.378744, 0.199373,
                -0.4839, -0.0772987, -0.0406905,
                0, 0.493875, 0.250495,
                -0.00533025, 0.577691, 0.320061,
                0.122905, 0.0250538, 0.0138807,
                0, -0.123021, 0.0389986;
            EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        }
        EXPECT_TRUE(test_jacobian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testContinuousJointPose)
{
    try
    {
        TEST_COUT << "ContinuousJointPose test";
        Initializer map("exotica/ContinuousJointPose", {{"Name", std::string("MyTask")}, {}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));
        EXPECT_TRUE(test_jacobian(problem));
        EXPECT_TRUE(test_hessian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testIMesh)
{
    try
    {
        TEST_COUT << "Interaction mesh test";
        Initializer map("exotica/InteractionMesh", {{"Name", std::string("MyTask")},
                                                    {"ReferenceFrame", std::string("base")},
                                                    {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("base")}}),
                                                                                              Initializer("Frame", {{"Link", std::string("link2")}}),
                                                                                              Initializer("Frame", {{"Link", std::string("endeff")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));

        int N = problem->N;
        int M = problem->length_Phi;
        int L = 5;
        Eigen::MatrixXd X(L, N);
        Eigen::MatrixXd Y(L, M);
        Eigen::MatrixXd jacobian(L * M, N);
        jacobian.setZero();
        Y.setZero();
        X.setZero();

        X << 0.278917, 0.51947, -0.813039, -0.730195, 0.0404201, -0.843536, -0.860187, -0.59069, -0.0771591, 0.639355, 0.146637, 0.511162, -0.896122, -0.684386, 0.999987;
        Y << -0.0115151, -0.00329772, -0.652131, -0.01588, -0.00454774, 0.000489, 0.0418479, 0.0119845, 0.906934,
            0.0647007, -0.0579245, -0.635726, 0.0879002, -0.0786944, 0.0262201, -0.230697, 0.206536, 0.836692,
            0.0846586, -0.098373, -0.626482, 0.111269, -0.129294, 0.0559699, -0.308935, 0.358981, 0.824647,
            -0.0715067, -0.0531681, -0.641823, -0.0962226, -0.0715454, 0.0264905, 0.261816, 0.194671, 0.879061,
            0.014318, -0.0178999, -0.646355, 0.0198797, -0.024853, 0.00185134, -0.0509673, 0.0637179, 0.869616;
        jacobian << 0.00329772, -0.194435, -0.112919,
            -0.0115151, -0.0556828, -0.0323379,
            0, 0.00993552, -0.0177924,
            0.00454774, -0.267977, -0.155057,
            -0.01588, -0.0767438, -0.0444055,
            0, 0.0165184, 0.00951859,
            -0.0119845, 0.706188, 0.414101,
            0.0418479, 0.202239, 0.118591,
            0, -0.0420476, 0.139591,
            0.0579245, -0.143241, -0.0744983,
            0.0647007, 0.128239, 0.066696,
            0, -0.0728714, -0.0644042,
            0.0786944, -0.18799, -0.100693,
            0.0879002, 0.168302, 0.0901469,
            0, -0.11798, -0.0656211,
            -0.206536, 0.493387, 0.232834,
            -0.230697, -0.441714, -0.208449,
            0, 0.298475, 0.326197,
            0.098373, -0.124335, -0.0691047,
            0.0846586, 0.144477, 0.0802994,
            0, -0.110572, -0.0639689,
            0.129294, -0.151303, -0.0843603,
            0.111269, 0.175813, 0.0980263,
            0, -0.17058, -0.0955839,
            -0.358981, 0.420088, 0.230469,
            -0.308935, -0.488141, -0.267804,
            0, 0.457396, 0.270273,
            0.0531681, -0.159255, -0.085326,
            -0.0715067, -0.118412, -0.0634434,
            0, 0.0748349, 0.0556152,
            0.0715454, -0.207141, -0.112843,
            -0.0962226, -0.154018, -0.0839033,
            0, 0.119906, 0.0666997,
            -0.194671, 0.56362, 0.285766,
            0.261816, 0.419075, 0.212479,
            0, -0.315274, -0.273879,
            0.0178999, -0.122936, -0.0735487,
            0.014318, 0.153692, 0.0919485,
            0, -0.0190144, 0.0184454,
            0.024853, -0.170294, -0.100978,
            0.0198797, 0.212897, 0.12624,
            0, -0.0318257, -0.0186769,
            -0.0637179, 0.436598, 0.267206,
            -0.0509673, -0.545823, -0.334054,
            0, 0.0786625, -0.152426;
        EXPECT_TRUE(test_values(X, Y, jacobian, problem));
        EXPECT_TRUE(test_jacobian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testPoint2Line)
{
    try
    {
        TEST_COUT << "PointToLine Test";
        Initializer map("exotica/PointToLine", {{"Name", std::string("MyTask")},
                                                // {"EndPoint", std::string("0.5 0.5 1")},
                                                {"EndPoint", std::string("0.5 0.5 0")},
                                                {"EndEffector", std::vector<Initializer>(
                                                                    {Initializer("Frame", {{"Link", std::string("endeff")},
                                                                                           {"LinkOffset", std::string("0.5 0 0.5")},
                                                                                           {"Base", std::string("base")},
                                                                                           {"BaseOffset", std::string("0.5 0.5 0")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));
        // TODO: Add test_values

        EXPECT_TRUE(test_jacobian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testPoint2Plane)
{
    try
    {
        {
            TEST_COUT << "PointToPlane Test - Align with world";
            Initializer map("exotica/PointToPlane", {{"Name", std::string("MyTask")},
                                                     {"EndPoint", std::string("1 2 3")},
                                                     {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
            EXPECT_TRUE(test_hessian(problem));
        }

        {
            TEST_COUT << "PointToPlane Test - Align with world with rotation";
            Initializer map("exotica/PointToPlane", {{"Name", std::string("MyTask")},
                                                     {"EndPoint", std::string("1 2 3")},
                                                     {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}, {"BaseOffset", std::string("0.1 0.1 0.1 0.7071 0 0.7071 0")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
            EXPECT_TRUE(test_hessian(problem));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testCollisionDistance)
{
    try
    {
        TEST_COUT << "CollisionDistance test";
        Initializer map("exotica/CollisionDistance", {{"Name", std::string("MyTask")},
                                                      {"CheckSelfCollision", true},
                                                      {"WorldMargin", 0.0},
                                                      {"RobotMargin", 0.0},
                                                      {}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map, "exotica/CollisionSceneFCLLatest");
        EXPECT_TRUE(test_random(problem));
        // EXPECT_TRUE(test_jacobian(problem, 1e-2));  // Currently failing
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testSmoothCollisionDistance)
{
    try
    {
        TEST_COUT << "SmoothCollisionDistance test";
        const std::vector<bool> linear_and_quadratic = {true, false};
        for (bool is_linear : linear_and_quadratic)
        {
            TEST_COUT << "Testing " << (is_linear ? "linear" : "quadratic") << " SmoothCollisionDistance";
            Initializer map("exotica/SmoothCollisionDistance", {{"Name", std::string("MyTask")},
                                                                {"CheckSelfCollision", false},
                                                                {"WorldMargin", 0.01},
                                                                {"RobotMargin", 0.01},
                                                                {"Linear", is_linear},
                                                                {}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map, "exotica/CollisionSceneFCLLatest");
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testQuasiStatic)
{
    try
    {
        {
            TEST_COUT << "QuasiStatic test inside capped";
            Initializer map("exotica/QuasiStatic", {
                                                       {"Name", std::string("MyTask")},
                                                       {"PositiveOnly", true},
                                                       {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-3 3 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-3 -3 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("3 -3 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("3 3 0")}})})},
                                                   });
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }

        {
            TEST_COUT << "QuasiStatic test outside capped";
            Initializer map("exotica/QuasiStatic", {
                                                       {"Name", std::string("MyTask")},
                                                       {"PositiveOnly", true},
                                                       {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-11 1 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-11 -1 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-10 -1 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-10 1 0")}})})},
                                                   });
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }

        {
            TEST_COUT << "QuasiStatic test inside";
            Initializer map("exotica/QuasiStatic", {
                                                       {"Name", std::string("MyTask")},
                                                       {"PositiveOnly", false},
                                                       {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-3 3 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-3 -3 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("3 -3 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("3 3 0")}})})},
                                                   });
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }

        {
            TEST_COUT << "QuasiStatic test outside";
            Initializer map("exotica/QuasiStatic", {
                                                       {"Name", std::string("MyTask")},
                                                       {"PositiveOnly", false},
                                                       {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-11 1 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-11 -1 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-10 -1 0")}}),
                                                                                                 Initializer("Frame", {{"Link", std::string("")}, {"LinkOffset", std::string("-10 1 0")}})})},
                                                   });
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testJointSmoothingBackwardDifference)
{
    try
    {
        std::vector<std::string> joint_smoothing_taskmaps = {"JointVelocityBackwardDifference", "JointAccelerationBackwardDifference", "JointJerkBackwardDifference"};
        for (const auto& smoothing_task : joint_smoothing_taskmaps)
        {
            {
                TEST_COUT << smoothing_task + " Test - test default initialisation";
                Initializer map("exotica/" + smoothing_task, {{"Name", std::string("MyTask")}, {"dt", 0.01}});
                UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
                EXPECT_TRUE(test_random(problem));
                EXPECT_TRUE(test_jacobian(problem));
            }

            // TODO(#437): Activate once solution for pointer casting/dynamic loading is found.
            // {
            //     TEST_COUT << smoothing_task + " Test - test SetPreviousJointState initialisation";
            //     Initializer map("exotica/" + smoothing_task, {{"Name", std::string("MyTask")}, {"dt", 0.01}});
            //     UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
            //     Eigen::VectorXd q_rand(problem->N);
            //     q_rand.setRandom();
            //     if (smoothing_task == "JointVelocityBackwardDifference")
            //     {
            //         std::shared_ptr<TaskMap> my_task = problem->GetTaskMaps()["MyTask"];
            //         std::shared_ptr<JointVelocityBackwardDifference> joint_velocity_smoothing_task = std::dynamic_pointer_cast<JointVelocityBackwardDifference>(my_task);
            //         joint_velocity_smoothing_task->SetPreviousJointState(q_rand);
            //     }
            //     else if (smoothing_task == "JointAccelerationBackwardDifference")
            //     {
            //         std::shared_ptr<TaskMap> my_task = problem->GetTaskMaps()["MyTask"];
            //         std::shared_ptr<JointAccelerationBackwardDifference> joint_acceleration_smoothing_task = std::dynamic_pointer_cast<JointAccelerationBackwardDifference>(my_task);
            //         joint_acceleration_smoothing_task->SetPreviousJointState(q_rand);
            //     }
            //     else if (smoothing_task == "JointJerkBackwardDifference")
            //     {
            //         std::shared_ptr<TaskMap> my_task = problem->GetTaskMaps()["MyTask"];
            //         std::shared_ptr<JointJerkBackwardDifference> joint_jerk_smoothing_task = std::dynamic_pointer_cast<JointJerkBackwardDifference>(my_task);
            //         joint_jerk_smoothing_task->SetPreviousJointState(q_rand);
            //     }
            //     EXPECT_TRUE(test_random(problem));
            //     EXPECT_TRUE(test_jacobian(problem));
            // }
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testAvoidLookAtSphere)
{
    try
    {
        {
            TEST_COUT << "AvoidLookAtSphere";

            // Custom links
            std::vector<Initializer> custom_links({Initializer("Link", {{"Name", std::string("AvoidLookAtPosition")},
                                                                        {"Transform", std::string("1 1 2")}})});

            // Setup taskmap
            Initializer map("exotica/AvoidLookAtSphere",
                            {{"Name", std::string("MyTask")},
                             {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("AvoidLookAtPosition")}, {"Base", std::string("")}})})},
                             {"UseAsCost", std::string("1")},
                             {"Radii", std::string("0.5")}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map, "", custom_links);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem, 2e-5));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testLookAt)
{
    try
    {
        {
            TEST_COUT << "LookAt";

            // Custom links
            std::vector<Initializer> custom_links({Initializer("Link", {{"Name", std::string("LookAtTarget")},
                                                                        {"Transform", std::string("1 1 2")}})});

            // Setup taskmap
            Initializer map("exotica/LookAt", {{"Name", std::string("MyTask")},
                                               {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("LookAtTarget")}, {"Base", std::string("endeff")}})})}});
            UnconstrainedEndPoseProblemPtr problem = setup_problem(map, "", custom_links);
            EXPECT_TRUE(test_random(problem));
            EXPECT_TRUE(test_jacobian(problem, 2e-5));
        }
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaTaskMaps, testManipulability)
{
    try
    {
        TEST_COUT << "Manipulability Test";
        Initializer map("exotica/Manipulability", {{"Name", std::string("MyTask")},
                                                   {"EndEffector", std::vector<Initializer>(
                                                                       {Initializer("Frame", {{"Link", std::string("endeff")},
                                                                                              {"LinkOffset", std::string("0.5 0 0.5")},
                                                                                              {"Base", std::string("base")},
                                                                                              {"BaseOffset", std::string("0.5 0.5 0")}})})}});
        UnconstrainedEndPoseProblemPtr problem = setup_problem(map);
        EXPECT_TRUE(test_random(problem));
        // TODO: Add test_values

        EXPECT_TRUE(test_jacobian(problem));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "EXOTica_test_maps");
    int ret = RUN_ALL_TESTS();
    Setup::Destroy();
    return ret;
}
