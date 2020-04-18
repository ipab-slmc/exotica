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

using namespace exotica;

void run()
{
    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));

    // Scene using joint group 'arm'
    Initializer scene("Scene", {{"Name", std::string("MyScene")},
                                {"JointGroup", std::string("arm")},
                                {"URDF", std::string("{exotica_examples}/resources/robots/lwr_simplified.urdf")},
                                {"SRDF", std::string("{exotica_examples}/resources/robots/lwr_simplified.srdf")}});
    // End-effector task map with two position frames
    Initializer map("exotica/EffFrame", {{"Name", std::string("Position")},
                                         {"EndEffector", std::vector<Initializer>({
                                                             Initializer("Frame", {{"Link", std::string("lwr_arm_6_link")}, {"LinkOffset", Eigen::VectorTransform(0, 0, 0, 0.7071067811865476, -4.3297802811774664e-17, 0.7071067811865475, 4.3297802811774664e-17)}}),
                                                         })}});
    Initializer cost("exotica/Task", {{"Task", std::string("Position")}});
    Eigen::VectorXd W(7);
    W << 7, 6, 5, 4, 3, 2, 1;
    Eigen::VectorXd start_state = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd nominal_state = Eigen::VectorXd::Zero(7);

    Initializer problem("exotica/UnconstrainedEndPoseProblem", {
                                                                   {"Name", std::string("MyProblem")},
                                                                   {"PlanningScene", scene},
                                                                   {"Maps", std::vector<Initializer>({map})},
                                                                   {"Cost", std::vector<Initializer>({cost})},
                                                                   {"W", W},
                                                                   {"Tolerance", 1e-5},
                                                                   {"StartState", start_state},
                                                                   {"NominalState", nominal_state},
                                                               });

    Initializer solver("exotica/IKSolver", {
                                               {"Name", std::string("MySolver")},
                                               {"MaxIterations", 1},
                                           });

    HIGHLIGHT_NAMED("GenericLoader", "Loaded from a hardcoded generic initializer.");

    // Initialize

    PlanningProblemPtr any_problem = Setup::CreateProblem(problem);
    MotionSolverPtr any_solver = Setup::CreateSolver(solver);

    // Assign the problem to the solver
    any_solver->SpecifyProblem(any_problem);
    UnconstrainedEndPoseProblemPtr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

    // Create the initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(any_problem->N);
    Eigen::MatrixXd solution;

    HIGHLIGHT("Calling solve() in an infinite loop");

    ros::Rate loop_rate(500.0);
    ros::WallTime init_time = ros::WallTime::now();

    while (ros::ok())
    {
        // Update the goal if necessary
        // e.g. figure eight
        const double t = ros::Duration((ros::WallTime::now() - init_time).toSec()).toSec();
        my_problem->cost.y = {0.6,
                              -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
                              0.5 + sin(t * M_PI * 0.5) * 0.2, 0, 0, 0};

        // Solve the problem using the IK solver
        my_problem->SetStartState(q);
        any_solver->Solve(solution);

        q = solution.row(solution.rows() - 1);

        my_problem->Update(q);
        my_problem->GetScene()->GetKinematicTree().PublishFrames();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // All classes will be destroyed at this point.
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_cpp_init_generic_node");
    HIGHLIGHT("Started");

    // Run demo code
    run();

    // Clean up
    // Run this only after all the exoica classes have been disposed of!
    Setup::Destroy();
}
