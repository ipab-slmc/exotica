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

std::string urdf_string = "<robot name=\"test_robot\"><link name=\"base\"><visual><geometry><cylinder length=\"0.3\" radius=\"0.2\"/></geometry><origin xyz=\"0 0 0.15\"/></visual></link><link name=\"link1\"> <visual><geometry><cylinder length=\"0.15\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.075\"/></visual> </link><link name=\"link2\"><visual><geometry><cylinder length=\"0.35\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.175\"/></visual> </link><link name=\"link3\"><visual><geometry><cylinder length=\"0.45\" radius=\"0.05\"/></geometry><origin xyz=\"0 0 0.225\"/></visual></link><link name=\"endeff\"><visual><geometry><cylinder length=\"0.05\" radius=\"0.1\"/></geometry><origin xyz=\"0 0 -0.025\"/></visual></link><joint name=\"joint1\" type=\"continuous\"><parent link=\"base\"/><child link=\"link1\"/><origin xyz=\"0 0 0.3\" rpy=\"0 0 0\" /><axis xyz=\"0 0 1\" /></joint><joint name=\"joint2\" type=\"continuous\"><parent link=\"link1\"/><child link=\"link2\"/><origin xyz=\"0 0 0.15\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /></joint><joint name=\"joint3\" type=\"continuous\"><parent link=\"link2\"/><child link=\"link3\"/><origin xyz=\"0 0 0.35\" rpy=\"0 0 0\" /><axis xyz=\"0 1 0\" /></joint><joint name=\"joint4\" type=\"fixed\"><parent link=\"link3\"/><child link=\"endeff\"/><origin xyz=\"0 0 0.45\" rpy=\"0 0 0\" /></joint></robot>";
std::string srdf_string = "<robot name=\"test_robot\"><group name=\"arm\"><chain base_link=\"base\" tip_link=\"endeff\" /></group><virtual_joint name=\"world_joint\" type=\"fixed\" parent_frame=\"world_frame\" child_link=\"base\" /><group_state name=\"zero\" group=\"arm\"><joint name=\"joint1\" value=\"0\" /><joint name=\"joint2\" value=\"0.3\" /><joint name=\"joint3\" value=\"0.55\" /></group_state><disable_collisions link1=\"base\" link2=\"link1\" reason=\"Adjacent\" /><disable_collisions link1=\"endeff\" link2=\"link3\" reason=\"Adjacent\" /><disable_collisions link1=\"link1\" link2=\"link2\" reason=\"Adjacent\" /><disable_collisions link1=\"link2\" link2=\"link3\" reason=\"Adjacent\" /></robot>";

bool testCore()
{
    if (Setup::getSolvers().size() == 0)
    {
        HIGHLIGHT_NAMED("EXOTica", "Failed to find any solvers.");
        return false;
    }
    if (Setup::getProblems().size() == 0)
    {
        HIGHLIGHT_NAMED("EXOTica", "Failed to find any problems.");
        return false;
    }
    if (Setup::getMaps().size() == 0)
    {
        HIGHLIGHT_NAMED("EXOTica", "Failed to find any maps.");
        return false;
    }
    return true;
}

bool testGenericInit()
{
    Initializer scene("Scene", {{"Name", std::string("MyScene")}, {"JointGroup", std::string("arm")}});
    Initializer map("exotica/EffPosition", {{"Name", std::string("Position")},
                                            {"Scene", std::string("MyScene")},
                                            {"EndEffector", std::vector<Initializer>({Initializer("Frame", {{"Link", std::string("endeff")}})})}});
    Eigen::VectorXd W(3);
    W << 3, 2, 1;
    Initializer problem("exotica/UnconstrainedEndPoseProblem", {
                                                                   {"Name", std::string("MyProblem")},
                                                                   {"PlanningScene", scene},
                                                                   {"Maps", std::vector<Initializer>({map})},
                                                                   {"W", W},
                                                               });
    Initializer solver("exotica/IKsolver", {
                                               {"Name", std::string("MySolver")},
                                               {"MaxIterations", 1},
                                               {"MaxStep", 0.1},
                                               {"C", 1e-3},
                                           });
    Server::Instance()->getModel("robot_description", urdf_string, srdf_string);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    return true;
}

bool testXMLInit()
{
    std::string XMLstring = "<IKSolverDemoConfig><IKsolver Name=\"MySolver\"><MaxIterations>1</MaxIterations><MaxStep>0.1</MaxStep><C>1e-3</C></IKsolver><UnconstrainedEndPoseProblem Name=\"MyProblem\"><PlanningScene><Scene Name=\"MyScene\"><JointGroup>arm</JointGroup></Scene></PlanningScene><Maps><EffPosition Name=\"Position\"><Scene>MyScene</Scene><EndEffector><Frame Link=\"endeff\" /></EndEffector></EffPosition></Maps><W> 3 2 1 </W></UnconstrainedEndPoseProblem></IKSolverDemoConfig>";
    Initializer solver, problem;
    XMLLoader::load(XMLstring, solver, problem, "", "", true);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    return true;
}

bool testRos()
{
    {
        HIGHLIGHT("Parsing EXOTica paths...");
        std::string path1 = ros::package::getPath("exotica");
        std::string path2 = parsePath("{exotica}");
        if (path1 != path2)
            throw_pretty("Failed when parsing paths:\n"
                         << path1 << "\n"
                         << path2);
    }

    // Reset server
    Server::destroy();

    if (Server::isRos()) throw_pretty("ROS node initialized, but it shouldn't have been!");

    // Fail when ROS node has not been initialized
    try
    {
        Server::Instance()->getNodeHandle();
        return false;
    }
    catch (Exception& e)
    {
    }

    if (Server::hasParam("/rosdistro")) throw_pretty("ROS param retrieved, but shouldn't have!");
    try
    {
        std::string param;
        Server::getParam("/rosdistro", param);
        return false;
    }
    catch (Exception& e)
    {
    }

    // Load robot model into cache from a file
    Server::Instance()->getModel("robot_description", urdf_string, srdf_string);
    // Load model from the cache
    Server::Instance()->getModel("robot_description");
    // Reset server, deleting the model cache
    Server::destroy();
    // Attempt to load model from the empty cache
    try
    {
        // Fails because URDF/SRDF are not specified and ROS node is not running to load model from ROS params.
        Server::Instance()->getModel("robot_description");
        return false;
    }
    catch (Exception& e)
    {
    }
    return true;
}

int main(int argc, char** argv)
{
    if (!testRos()) exit(2);
    HIGHLIGHT_NAMED("EXOTica", "ROS test passed.");
    ros::init(argc, argv, "EXOTica_test_initializers");
    if (!testCore()) exit(2);
    HIGHLIGHT_NAMED("EXOTica", "Core test passed.");
    if (!testGenericInit()) exit(2);
    HIGHLIGHT_NAMED("EXOTica", "Generic initialization test passed.");
    if (!testXMLInit()) exit(2);
    HIGHLIGHT_NAMED("EXOTica", "XML initialization test passed.");
    Setup::Destroy();
    return 0;
}
