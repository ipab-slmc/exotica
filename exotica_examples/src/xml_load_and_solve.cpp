//
// Copyright (c) 2020, University of Oxford
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
#include <ros/ros.h>

using namespace exotica;

int main(int argc, char **argv)
{
    if (argc != 2) ThrowPretty("No file argument provided.");

    std::string file_name = argv[1];
    HIGHLIGHT("Loading from " << file_name);
    // Server::GetParam("ConfigurationFile", file_name);

    if (file_name.empty()) ThrowPretty("ConfigurationFile not specified!");

    ros::init(argc, argv, "example_cpp_load_and_solve_node");
    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));
    MotionSolverPtr solver = XMLLoader::LoadSolver(file_name);

    Eigen::MatrixXd solution;
    solver->Solve(solution);
    HIGHLIGHT("Solved in " << solver->GetPlanningTime() << "s");

    solver.reset();

    // Clean up
    // Run this only after all the exotica classes have been disposed of!
    Setup::Destroy();
}
