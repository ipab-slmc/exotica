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
#include <exotica/Problems/UnconstrainedEndPoseProblem.h>

using namespace exotica;

void run()
{

    Initializer solver, problem;

    std::string file_name = ros::package::getPath("exotica_examples")+"/resources/ik_solver_demo.xml";

    XMLLoader::load(file_name, solver, problem);

    HIGHLIGHT_NAMED("XMLnode","Loaded from XML");

    UnconstrainedEndPoseProblemInitializer problem_init(problem);
    problem_init.PlanningScene.addProperty( Property("URDF", false, boost::any(ros::package::getPath("exotica_examples")+"/resources/lwr_simplified.urdf")));
    problem_init.PlanningScene.addProperty( Property("SRDF", false, boost::any(ros::package::getPath("exotica_examples")+"/resources/lwr_simplified.srdf")));

    PlanningProblem_ptr any_problem = Setup::createProblem(problem_init);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);

    // Assign the problem to the solver
    any_solver->specifyProblem(any_problem);
    UnconstrainedEndPoseProblem_ptr my_problem = std::static_pointer_cast<UnconstrainedEndPoseProblem>(any_problem);

    // Create the initial configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(my_problem->N);
    Eigen::MatrixXd solution;

    my_problem->qNominal = q;

    ROS_INFO_STREAM("Calling solve() in an infinite loop");

    double t = 0.0;
    double dt = 1.0/20.0;

    while (true)
    {
        Timer timer;
        // Update the goal if necessary
        // e.g. figure eight
        t += dt;
        my_problem->y = {0.6,
                -0.1 + sin(t * 2.0 * M_PI * 0.5) * 0.1,
                0.5 + sin(t * M_PI * 0.5) * 0.2 ,0 ,0 ,0};

        // Solve the problem using the IK solver
        my_problem->setStartState(q);
        any_solver->Solve(solution);

        HIGHLIGHT("Finished solving in "<<timer.getDuration()<<"s. Solution ["<<solution<<"]");
        q = solution.row(0);

        my_problem->Update(q);

        exotica::sleep(dt);
    }
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Started");

    // Run demo code
    run();
}
