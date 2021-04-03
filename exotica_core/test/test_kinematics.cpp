//
// Copyright (c) 2020, University of Edinburgh
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
#include <exotica_core/tools/test_helpers.h>
#include <gtest/gtest.h>

using namespace exotica;

static const std::string urdf_string_ = "<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual><collision><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></collision></link><link name=\"link1\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual><collision><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></collision></link><link name=\"link2\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual><collision><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></collision></link><link name=\"link3\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual><collision><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></collision></link><link name=\"endeff\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual><collision><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></collision></link><joint name=\"joint1\" type=\"revolute\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /><limit effort=\"200\" velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint2\" type=\"revolute\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\" velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint3\" type=\"revolute\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\" velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
static const std::string srdf_string_ = "<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";

constexpr bool print_debug_information_ = false;
constexpr int num_trials_ = 100;

class TestClass
{
public:
    ScenePtr scene;
    KinematicSolution solution;
    int N;

    void UpdateKinematics(std::shared_ptr<KinematicResponse> response)
    {
        solution.Create(response);
    }

    TestClass()
    {
        Server::Instance()->GetModel("robot_description", urdf_string_, srdf_string_);
        scene.reset(new Scene());
        Initializer init("Scene", {
                                      {"Name", std::string("MyScene")},
                                      {"JointGroup", std::string("arm")},
                                      {"DoNotInstantiateCollisionScene", std::string("1")},
                                  });
        scene->InstantiateInternal(SceneInitializer(init));

        KinematicsRequest request;
        request.flags = KIN_FK | KIN_J | KIN_H;
        request.frames = {KinematicFrameRequest("endeff")};
        solution = KinematicSolution(0, 1);
        scene->RequestKinematics(request, std::bind(&TestClass::UpdateKinematics, this, std::placeholders::_1));

        N = scene->GetKinematicTree().GetNumControlledJoints();
    }
};

bool test_jacobian(TestClass& test, const double eps = 1e-5)
{
    constexpr double h = 1e-5;

    TEST_COUT << "Testing Jacobian with h=" << h << ", eps=" << eps;
    for (int k = 0; k < num_trials_; ++k)
    {
        Eigen::VectorXd x0(test.N);
        x0 = test.scene->GetKinematicTree().GetRandomControlledState();
        test.scene->Update(x0, 0.0);
        KDL::Frame y0 = test.solution.Phi(0);
        KDL::Jacobian J0 = test.solution.jacobian(0);
        KDL::Jacobian jacobian(test.N);
        jacobian.data.setZero();
        for (int i = 0; i < test.N; ++i)
        {
            Eigen::VectorXd x(x0);
            x(i) += h;
            test.scene->Update(x, 0.0);
            KDL::Frame y = test.solution.Phi(0);
            KDL::Twist diff = KDL::diff(y0, y);
            for (int j = 0; j < 6; ++j)
                jacobian.data(j, i) = diff[j] / h;
        }
        double errJ = (jacobian.data - J0.data).norm();
        if (errJ > eps)
        {
            TEST_COUT << "x: " << x0.transpose();
            TEST_COUT << "J*:\n"
                      << jacobian.data;
            TEST_COUT << "J:\n"
                      << J0.data;
            TEST_COUT << "(J*-J):\n"
                      << (jacobian.data - J0.data);
            ADD_FAILURE() << "Jacobian error out of bounds: " << errJ;
        }
    }
    return true;
}

bool test_hessian(TestClass& test, const double eps = 1e-5)
{
    constexpr double h = 1e-5;

    TEST_COUT << "Testing Hessian with h=" << h << ", eps=" << eps;
    for (int l = 0; l < num_trials_; ++l)
    {
        Eigen::VectorXd x0(test.N);
        x0 = test.scene->GetKinematicTree().GetRandomControlledState();
        test.scene->Update(x0, 0.0);
        Hessian H0 = test.solution.hessian(0);
        Hessian hessian = Hessian::Constant(6, Eigen::MatrixXd::Zero(test.N, test.N));
        Eigen::VectorXd x;
        for (int j = 0; j < test.N; ++j)
        {
            x = x0;
            x(j) += h;
            test.scene->Update(x, 0.0);
            Eigen::MatrixXd J1 = test.solution.jacobian(0).data;
            x = x0;
            x(j) -= h;
            test.scene->Update(x, 0.0);
            Eigen::MatrixXd J2 = test.solution.jacobian(0).data;
            for (int i = 0; i < test.N; ++i)
            {
                for (int k = 0; k < 6; ++k)
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
            TEST_COUT << "H*:\n"
                      << hessian(0)
                      << "\n\n"
                      << hessian(1)
                      << "\n\n"
                      << hessian(2)
                      << "\n\n"
                      << hessian(3)
                      << "\n\n"
                      << hessian(4)
                      << "\n\n"
                      << hessian(5)
                      << "\n\n...";
            TEST_COUT << "H:\n"
                      << H0(0)
                      << "\n\n"
                      << H0(1)
                      << "\n\n"
                      << H0(2)
                      << "\n\n"
                      << H0(3)
                      << "\n\n"
                      << H0(4)
                      << "\n\n"
                      << H0(5)
                      << "\n...";
            ADD_FAILURE() << "Hessian error out of bounds: " << errH;
        }
    }
    return true;
}

TEST(ExoticaCore, testKinematicJacobian)
{
    try
    {
        TEST_COUT << "Kinematic Jacobian test";
        TestClass test;
        EXPECT_TRUE(test_jacobian(test));
    }
    catch (const std::exception& e)
    {
        ADD_FAILURE() << "Uncaught exception! " << e.what();
    }
}

TEST(ExoticaCore, testKinematicHessian)
{
    try
    {
        TEST_COUT << "Kinematic Hessian test";
        TestClass test;
        EXPECT_TRUE(test_hessian(test));
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
