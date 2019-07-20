//
// Copyright (c) 2019, University of Edinburgh
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

#include <exotica_control_rrt_solver/control_rrt_solver.h>

REGISTER_MOTIONSOLVER_TYPE("ControlRRTSolver", exotica::ControlRRTSolver)

namespace exotica
{
void ControlRRTSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::DynamicTimeIndexedShootingProblem")
    {
        ThrowNamed("This ControlRRTSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(pointer);
    dynamics_solver_ = prob_->GetScene()->GetDynamicsSolver();
    if (debug_) HIGHLIGHT_NAMED("ControlRRTSolver", "initialized");
}

void ControlRRTSolver::Setup()
{
    int T = prob_->get_T();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();
    const double dt = dynamics_solver_->get_dt();
    if (parameters_.Seed != -1) ompl::RNG::setSeed(parameters_.Seed);
    
    std::shared_ptr<ob::RealVectorStateSpace> space(
        std::make_shared<ob::RealVectorStateSpace>(NX)
    );

    state_bounds_ = std::make_shared<ob::RealVectorBounds>(NX);
    
    for (int i = 0; i < NX; ++ i)
    {
        state_bounds_->setLow(i, -parameters_.StateLimits(i));
        state_bounds_->setHigh(i, parameters_.StateLimits(i));
    }

    space->setBounds(*state_bounds_);

    // create a control space
    std::shared_ptr<oc::RealVectorControlSpace> cspace(
        std::make_shared<oc::RealVectorControlSpace>(space, NU)
    );

    // set the bounds for the control space
    control_bounds_ = std::make_shared<ob::RealVectorBounds>(NU);
    const Eigen::VectorXd control_limits = dynamics_solver_->get_control_limits();
    
    for (int i = 0; i < NU; ++ i)
    {
        control_bounds_->setLow(i, -control_limits(i));
        control_bounds_->setHigh(i, control_limits(i));
    }

    cspace->setBounds(*control_bounds_);

    // define a simple setup class
    setup_ = std::make_shared<oc::SimpleSetup>(cspace);
    oc::SpaceInformationPtr si = setup_->getSpaceInformation();

    si->setPropagationStepSize(dt);
    setup_->setStateValidityChecker([this, si](const ob::State *state) { return isStateValid(si, state); });

    std::shared_ptr<OMPLStatePropagator> propagator(
        std::make_shared<OMPLStatePropagator>(si, dynamics_solver_)
    );

    setup_->setStatePropagator(propagator);

    const Eigen::VectorXd start_eig = prob_->get_X().col(0);
    const Eigen::MatrixXd goal_eig = prob_->get_X_star().col(T - 1);
    
    start_state_ = std::make_shared<ob::ScopedState<ob::RealVectorStateSpace>>(space);
    goal_state_ = std::make_shared<ob::ScopedState<ob::RealVectorStateSpace>>(space);

    for (int i = 0; i < NX; ++ i)
    {
        (*start_state_)[i] = start_eig(i);
        (*goal_state_)[i] = goal_eig(i);
    }

    setup_->setStartAndGoalStates(*start_state_, *goal_state_, parameters_.ConvergenceTolerance);

    ob::PlannerPtr optimizingPlanner(new ompl::control::RRT(si));
    setup_->setPlanner(optimizingPlanner);

    setup_->setup();
    propagator->setIntegrationTimeStep(si->getPropagationStepSize());

}

void ControlRRTSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    Timer planning_timer, backward_pass_timer, line_search_timer;

    int T = prob_->get_T();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();
    const Eigen::VectorXd x_star = prob_->get_X_star().col(T - 1);

    const double dt = dynamics_solver_->get_dt();
    
    Setup();
    ob::PlannerStatus solved = setup_->solve(parameters_.MaxIterationTime);

    if (solved)
    {
        std::vector<oc::Control*> controls = setup_->getSolutionPath().getControls();
        std::vector<ob::State*> states = setup_->getSolutionPath().getStates();
        std::vector<double> durations = setup_->getSolutionPath().getControlDurations();

        Eigen::VectorXd sol_goal_state = Eigen::Map<Eigen::VectorXd>(
            states.back()->as<ob::RealVectorStateSpace::StateType>()->values, NX
        );

        T = 0;
        for (int t = 0; t < controls.size(); ++t)
            T += (int)(durations[t] / dt);
        T += 1;

        if ((x_star - sol_goal_state).norm() > parameters_.ConvergenceTolerance && debug_)
            WARNING_NAMED("ControlRRTSolver", "Goal not satisfied.");

        // additional solution criteria
        if (T <= 2 || (
            (x_star - sol_goal_state).norm() > parameters_.ConvergenceTolerance && 
            !parameters_.ApproximateSolution
        )) {
            if (debug_) HIGHLIGHT_NAMED("ControlRRTSolver", "No solution found.");
            return;
        } else if(debug_) HIGHLIGHT_NAMED("ControlRRTSolver", "Found solution.");

        prob_->set_T(T);
        prob_->PreUpdate();

        solution.resize(T, NU);

        int t = 0;
        for (int i = 0; i < controls.size(); ++i)
        {
            double *oc_u = controls[i]->as<oc::RealVectorControlSpace::ControlType>()->values;
            double *oc_s = states[i]->as<ob::RealVectorStateSpace::StateType>()->values;
                
            Eigen::VectorXd u = Eigen::Map<Eigen::VectorXd>(oc_u, NU);

            for (int k = 0; k < (int)(durations[i] / dt); ++ k)
            {
                solution.row(t) = u.transpose();
                prob_->Update(u, t);

                ++t;
            }
        }

        planning_time_ = planning_timer.GetDuration();
    }
    else
    {
        HIGHLIGHT_NAMED("ControlRRTSolver", "No solution found.");
    }
}

}  // namespace exotica
