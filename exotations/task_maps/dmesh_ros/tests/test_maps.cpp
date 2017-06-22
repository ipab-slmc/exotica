/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of  nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <exotica/Exotica.h>

using namespace exotica;

std::string urdf_string="<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual></link><link name=\"link1\"> <inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual> </link><link name=\"link2\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual> </link><link name=\"link3\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual></link><link name=\"endeff\"><inertial><mass value=\"0.2\"/><origin xyz=\"0 0 0.1\"/><inertia ixx=\"0.00381666666667\" ixy=\"0\" ixz=\"0\" iyy=\"0.0036\" iyz=\"0\" izz=\"0.00381666666667\"/></inertial><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual></link><joint name=\"joint1\" type=\"revolute\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint2\" type=\"revolute\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint3\" type=\"revolute\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /><limit effort=\"200\"  velocity=\"1.0\" lower=\"-0.4\" upper=\"0.4\"/><safety_controller k_position=\"30\" k_velocity=\"30\" soft_lower_limit=\"-0.4\" soft_upper_limit=\"0.4\"/></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
std::string srdf_string="<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";
bool printInfo = true;

#define NUM_TRIALS 100

void testRandom(UnconstrainedEndPoseProblem_ptr problem)
{
    Eigen::VectorXd x(3);
    INFO_PLAIN("Testing random configurations:")
    for(int i=0;i<NUM_TRIALS;i++)
    {
        x.setRandom();
        problem->Update(x);
        if(printInfo)
        {
            INFO_PLAIN("x = "<<x.transpose());
            INFO_PLAIN("y = "<<problem->Phi.transpose());
            INFO_PLAIN("J = \n"<<problem->J);
        }
    }
    INFO_PLAIN("Test passed");
}

void testValues(Eigen::MatrixXdRefConst Xref, Eigen::MatrixXdRefConst Yref, Eigen::MatrixXdRefConst Jref, UnconstrainedEndPoseProblem_ptr problem, double eps = 1e-5)
{
    INFO_PLAIN("Testing set points:");
    int N = Xref.cols();
    int M = Yref.cols();
    int L = Xref.rows();
    for(int i=0;i<L;i++)
    {
        Eigen::VectorXd x = Xref.row(i);
        Eigen::VectorXd y = Yref.row(i);
        Eigen::MatrixXd J = Jref.middleRows(i*M,M);
        problem->Update(x);
        double errY = (y - problem->Phi).norm();
        double errJ = (J - problem->J).norm();
        if(errY > eps) throw_pretty("Task space error out of bounds: " << errY);
        if(errJ > eps)
        {
            HIGHLIGHT("x: "<<x.transpose());
            HIGHLIGHT("J*:\n"<<J);
            HIGHLIGHT("J:\n"<<problem->J);
            throw_pretty("Jacobian error out of bounds: " << errJ);
        }
    }

    INFO_PLAIN("Test passed");
}

void testJacobian(UnconstrainedEndPoseProblem_ptr problem, double eps = 1e-5)
{
    INFO_PLAIN("Testing Jacobian:");
    for(int j=0;j<NUM_TRIALS;j++)
    {
        Eigen::VectorXd x0(problem->N);
        x0.setRandom();
        problem->Update(x0);
        Eigen::VectorXd y0 = problem->Phi;
        Eigen::MatrixXd J0 = problem->J;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(J0.rows(), J0.cols());
        for(int i=0;i<problem->N;i++)
        {
            Eigen::VectorXd x = x0;
            x(i)+=eps;
            problem->Update(x);
            J.col(i) = (problem->Phi - y0)/eps;
        }
        double errJ = (J-J0).norm();
        if(errJ > eps) throw_pretty("Jacobian error out of bounds: " << errJ);
    }
    INFO_PLAIN("Test passed");
}

UnconstrainedEndPoseProblem_ptr setupProbelm(Initializer& map)
{
    Initializer scene("Scene",{{"Name",std::string("MyScene")},{"JointGroup",std::string("arm")}});
    map.addProperty(Property("Scene", true, std::string("MyScene")));
    Eigen::VectorXd W(3);W << 3,2,1;
    Initializer problem("exotica/UnconstrainedEndPoseProblem",{
                            {"Name",std::string("MyProblem")},
                            {"PlanningScene",scene},
                            {"Maps",std::vector<Initializer>({map})},
                            {"W",W},
                        });
    Server::Instance()->getModel("robot_description",urdf_string,srdf_string);
    return boost::static_pointer_cast<UnconstrainedEndPoseProblem>(Setup::createProblem(problem));
}

void testDMesh()
{
    ros::NodeHandle nh;
    nh.setParam("/MeshGraphManager/InteractRange", 1.0);
    nh.setParam("/MeshGraphManager/SafetyThreshold", 1e-3);
    nh.setParam("/MeshGraphManager/DummyTable", false);
    nh.setParam("/MeshGraphManager/ExternalTopic", "/MeshGraphManager/Updates");
    HIGHLIGHT("DMesh test");
    Initializer map("exotica/DMeshROS",{
                        {"Name",std::string("DMeshROS")},
                        {"PoseGain",0.1},
                        {"ObstacleGain",1.0},
                        {"GoalGain",0.1},
                        {"UsePose",true},
                        {"EndEffector",std::vector<Initializer>({
                             Initializer("Frame",{{"Name",std::string("A")}, {"LinkType",std::string("LINK")}, {"ConnectTo",std::string("B C D E")}, {"Link",std::string("base")}, {"Radius", 0.4} }),
                             Initializer("Frame",{{"Name",std::string("B")}, {"LinkType",std::string("LINK")}, {"ConnectTo",std::string("A C D E")}, {"Link",std::string("endeff")}, {"Radius", 0.4} }),
                             Initializer("Frame",{{"Name",std::string("C")}, {"LinkType",std::string("OBSTACLE")}, {"ConnectTo",std::string("B A D E")}, {"Link",std::string("base")}, {"Radius", 0.2}, {"LinkOffset", std::string("0.4 0.0 0.7 0.0 0.0 0.0 1.0")}}),
                             Initializer("Frame",{{"Name",std::string("D")}, {"LinkType",std::string("OBSTACLE")}, {"ConnectTo",std::string("B C A E")}, {"Link",std::string("base")}, {"Radius", 0.2}, {"LinkOffset", std::string("0.0 0.4 0.7 0.0 0.0 0.0 1.0")}}),
                             Initializer("Frame",{{"Name",std::string("E")}, {"LinkType",std::string("GOAL")}, {"ConnectTo",std::string("B C D A")}, {"Link",std::string("base")}, {"Radius", 0.2}, {"LinkOffset", std::string("0.0 -0.4 1.0 0.0 0.0 0.0 1.0")}})
                                        }) } });
    UnconstrainedEndPoseProblem_ptr problem = setupProbelm(map);
    testRandom(problem);

    int N = problem->N;
    int M = problem->PhiN;
    int L = 5;
    Eigen::MatrixXd X(L, N);
    Eigen::MatrixXd Y(L, M);
    Eigen::MatrixXd J(L*M, N);

/*    X << 0.680375, -0.211234, 0.566198, 0.59688, 0.823295, -0.604897, -0.329554, 0.536459, -0.444451, 0.10794, -0.0452059, 0.257742, -0.270431, 0.0268018, 0.904459;
    Y << 0.0645323, 0.0522249, 1.21417, 0.292945, 0.199075, 1.12724, 0.208378, -0.0712708, 1.19893, 0.0786457, 0.00852213, 1.23952, 0.356984, -0.0989639, 1.06844;
    J << -0.0522249, 0.594015, 0.327994
            , 0.0645323, 0.480726, 0.26544
            , 0, -0.0830172, -0.156401
            , -0.199075, 0.560144, 0.363351
            , 0.292945, 0.380655, 0.246921
            , 0, -0.354186, -0.0974994
            , 0.0712708, 0.708627, 0.423983
            , 0.208378, -0.24237, -0.145014
            , 0, -0.220229, -0.0413455
            ,-0.00852213, 0.784922, 0.437315
            , 0.0786457, 0.085055, 0.0473879
            , 0, -0.0791061, -0.0949228
            , 0.0989639, 0.595968, 0.258809
            , 0.356984, -0.165215, -0.0717477
            , 0, -0.370448, -0.361068;
    testValues(X ,Y ,J ,problem);*/

    testJacobian(problem);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EXOTica_test_maps");
    testDMesh();
    Setup::Destroy();
    return 0;
}
