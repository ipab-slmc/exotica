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

#ifndef EXOTICA_OMPL_CONTROL_SOLVER_OMPL_CONTROL_SOLVER_H_
#define EXOTICA_OMPL_CONTROL_SOLVER_OMPL_CONTROL_SOLVER_H_

// #include <exotica_core/feedback_motion_solver.h>
#include <exotica_core/motion_solver.h>
#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>

#include <exotica_ompl_control_solver/ompl_control_solver_initializer.h>

// TODO: Remove unused includes
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

typedef std::function<ob::PlannerPtr(const oc::SpaceInformationPtr &si)> ConfiguredPlannerAllocator;

namespace exotica
{
class OMPLStatePropagator : public oc::StatePropagator
{
public:
    OMPLStatePropagator(
        oc::SpaceInformationPtr si,
        DynamicsSolverPtr dynamics_solver_) : oc::StatePropagator(si), space_(si), dynamics_solver_(dynamics_solver_) {}
    void propagate(
        const ob::State *state,
        const oc::Control *control,
        const double duration,
        ob::State *result) const override
    {
        double t = 0;
        space_->copyState(result, state);

        while (t < duration)
        {
            Integrate(result, control, timeStep_);
            t += timeStep_;
        }
    }

    void setIntegrationTimeStep(double timeStep)
    {
        timeStep_ = timeStep;
    }

    double getIntegrationTimeStep() const
    {
        return timeStep_;
    }

private:
    double timeStep_ = 0.0;
    oc::SpaceInformationPtr space_;
    DynamicsSolverPtr dynamics_solver_;

    void Integrate(ob::State *ob_x, const oc::Control *oc_u, double dt) const
    {
        const int NU = dynamics_solver_->get_num_controls();
        const int NX = dynamics_solver_->get_num_positions() + dynamics_solver_->get_num_velocities();

        double *x = ob_x->as<ob::RealVectorStateSpace::StateType>()->values;
        double *u = oc_u->as<oc::RealVectorControlSpace::ControlType>()->values;

        Eigen::VectorXd eig_x = Eigen::Map<Eigen::VectorXd>(x, NX);
        Eigen::VectorXd eig_u = Eigen::Map<Eigen::VectorXd>(u, NU);

        Eigen::VectorXd x_new = dynamics_solver_->Simulate(eig_x, eig_u, dt);
        std::memcpy(x, x_new.data(), NX * sizeof(double));
    }
};

class OMPLControlSolver : public MotionSolver
{
public:
    ///\brief Solves the problem
    ///@param solution Returned solution trajectory as a vector of joint configurations.
    void Solve(Eigen::MatrixXd &solution) override;

    ///\brief Binds the solver to a specific problem which must be pre-initalised
    ///@param pointer Shared pointer to the motion planning problem
    ///@return        Successful if the problem is a valid DynamicTimeIndexedProblem
    void SpecifyProblem(PlanningProblemPtr pointer) override;

protected:
    template <typename PlannerType>
    static ob::PlannerPtr AllocatePlanner(const oc::SpaceInformationPtr &si)
    {
        ob::PlannerPtr planner(new PlannerType(si));
        return planner;
    }

    DynamicTimeIndexedShootingProblemPtr prob_;  ///!< Shared pointer to the planning problem.
    DynamicsSolverPtr dynamics_solver_;          ///!< Shared pointer to the dynamics solver.

    OMPLControlSolverInitializer init_;

    std::unique_ptr<oc::SimpleSetup> setup_;
    std::string algorithm_;
    ConfiguredPlannerAllocator planner_allocator_;

    void Setup();

    bool isStateValid(const oc::SpaceInformationPtr si, const ob::State *state)
    {
        return true;
    }
};

}  // namespace exotica

#endif  // EXOTICA_OMPL_CONTROL_SOLVER_OMPL_CONTROL_SOLVER_H_
