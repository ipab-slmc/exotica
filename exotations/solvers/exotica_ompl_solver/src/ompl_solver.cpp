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

#include <exotica_ompl_solver/ompl_solver.h>

#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

namespace exotica
{
template <class ProblemType>
OMPLSolver<ProblemType>::OMPLSolver() = default;

template <class ProblemType>
OMPLSolver<ProblemType>::~OMPLSolver() = default;

template <class ProblemType>
void OMPLSolver<ProblemType>::SpecifyProblem(PlanningProblemPtr pointer)
{
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<ProblemType>(pointer);
    if (prob_->GetScene()->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
        state_space_.reset(new OMPLRNStateSpace(init_));
    else if (prob_->GetScene()->GetKinematicTree().GetControlledBaseType() == BaseType::PLANAR)
    {
        if (init_.IsDubinsStateSpace)
            state_space_.reset(new OMPLDubinsRNStateSpace(init_));
        else
            state_space_.reset(new OMPLRNStateSpace(init_));  // NB: We have a dedicated OMPLSE2RNStateSpace, however, for now we cannot set orientation bounds on it - as thus we are using the RN here. Cf. issue #629.
    }
    else if (prob_->GetScene()->GetKinematicTree().GetControlledBaseType() == BaseType::FLOATING)
        state_space_.reset(new OMPLSE3RNStateSpace(init_));
    else
        ThrowNamed("Unsupported base type " << prob_->GetScene()->GetKinematicTree().GetControlledBaseType());
    ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(state_space_));
    ompl_simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new OMPLStateValidityChecker(ompl_simple_setup_->getSpaceInformation(), prob_)));
    ompl_simple_setup_->setPlannerAllocator(boost::bind(planner_allocator_, _1, algorithm_));

    if (init_.Projection.rows() > 0)
    {
        std::vector<int> project_vars(init_.Projection.rows());
        for (int i = 0; i < init_.Projection.rows(); ++i)
        {
            project_vars[i] = (int)init_.Projection(i);
            if (project_vars[i] < 0 || project_vars[i] >= prob_->N) ThrowNamed("Invalid projection index! " << project_vars[i]);
        }
        if (prob_->GetScene()->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
            ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLRNProjection(state_space_, project_vars)));
        else if (prob_->GetScene()->GetKinematicTree().GetControlledBaseType() == BaseType::PLANAR)
            ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLSE2RNProjection(state_space_, project_vars)));
        else if (prob_->GetScene()->GetKinematicTree().GetControlledBaseType() == BaseType::FLOATING)
            ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLSE3RNProjection(state_space_, project_vars)));
    }
}

template <class ProblemType>
int OMPLSolver<ProblemType>::GetRandomSeed() const
{
    return ompl::RNG::getSeed();
}

template <class ProblemType>
void OMPLSolver<ProblemType>::PreSolve()
{
    // Clear previously computed solutions
    if (!multi_query_)
    {
        ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
        const ompl::base::PlannerPtr planner = ompl_simple_setup_->getPlanner();
        if (planner)
            planner->clear();
        ompl_simple_setup_->getPlanner()->setProblemDefinition(ompl_simple_setup_->getProblemDefinition());
    }
    ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
}

template <class ProblemType>
void OMPLSolver<ProblemType>::PostSolve()
{
    ompl_simple_setup_->clearStartStates();
    int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
    int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
    if (debug_) CONSOLE_BRIDGE_logDebug("There were %d valid motions and %d invalid motions.", v, iv);

    if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
        CONSOLE_BRIDGE_logWarn("Computed solution is approximate");
}

template <class ProblemType>
void OMPLSolver<ProblemType>::SetGoalState(Eigen::VectorXdRefConst qT, const double eps)
{
    ompl::base::ScopedState<> gs(state_space_);
    state_space_->as<OMPLStateSpace>()->ExoticaToOMPLState(qT, gs.get());
    if (!ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
    {
        ThrowNamed("Goal state is not valid!");
    }

    if (!ompl_simple_setup_->getSpaceInformation()->satisfiesBounds(gs.get()))
    {
        state_space_->as<OMPLStateSpace>()->StateDebug(qT);

        // Debug state and bounds
        auto bounds = prob_->GetBounds();
        std::string out_of_bounds_joint_ids = "";
        for (int i = 0; i < qT.rows(); ++i)
            if (qT(i) < bounds[i] || qT(i) > bounds[i + qT.rows()])
                out_of_bounds_joint_ids += "[j" + std::to_string(i) + "=" + std::to_string(qT(i)) + ", ll=" + std::to_string(bounds[i]) + ", ul=" + std::to_string(bounds[i + qT.rows()]) + "]\n";

        ThrowNamed("Invalid goal state [Invalid joint bounds for joint indices: \n"
                   << out_of_bounds_joint_ids << "]");
    }
    ompl_simple_setup_->setGoalState(gs, eps);
}

template <class ProblemType>
void OMPLSolver<ProblemType>::GetPath(Eigen::MatrixXd &traj, ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::geometric::PathSimplifierPtr psf = ompl_simple_setup_->getPathSimplifier();
    const ompl::base::SpaceInformationPtr &si = ompl_simple_setup_->getSpaceInformation();

    ompl::geometric::PathGeometric pg = ompl_simple_setup_->getSolutionPath();
    if (init_.Smooth)
    {
        bool try_more = true;
        int times = 0;
        while (init_.ReduceVertices && times < init_.SimplifyTryCnt && try_more && ptc == false)
        {
            pg.interpolate(init_.SimplifyInterpolationLength);
            try_more = psf->reduceVertices(pg, 0, 0, init_.RangeRatio);
            ++times;
        }
        if (init_.ShortcutPath && si->getStateSpace()->isMetricSpace())
        {
            times = 0;
            while (times < init_.SimplifyTryCnt && try_more && ptc == false)
            {
                pg.interpolate(init_.SimplifyInterpolationLength);
                try_more = psf->shortcutPath(pg, 0, 0, init_.RangeRatio, init_.SnapToVertex);
                ++times;
            }
        }
    }
    std::vector<ompl::base::State *> &states = pg.getStates();
    unsigned int length = 0;
    if (init_.FinalInterpolationLength > 3)
    {
        length = init_.FinalInterpolationLength;
    }
    else
    {
        const int n1 = states.size() - 1;
        for (int i = 0; i < n1; ++i)
            length += si->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
    }
    pg.interpolate(int(length * init_.SmoothnessFactor));

    traj.resize(pg.getStateCount(), prob_->GetSpaceDim());
    Eigen::VectorXd tmp(prob_->GetSpaceDim());

    for (int i = 0; i < static_cast<int>(pg.getStateCount()); ++i)
    {
        state_space_->as<OMPLStateSpace>()->OMPLToExoticaState(pg.getState(i), tmp);
        traj.row(i) = tmp.transpose();
    }
}

template <class ProblemType>
void OMPLSolver<ProblemType>::Solve(Eigen::MatrixXd &solution)
{
    // Set log level
    ompl::msg::setLogLevel(debug_ ? ompl::msg::LogLevel::LOG_DEBUG : ompl::msg::LogLevel::LOG_WARN);

    Eigen::VectorXd q0 = prob_->ApplyStartState();

    // check joint limits
    const std::vector<double> bounds = prob_->GetBounds();
    for (const double l : bounds)
    {
        if (!std::isfinite(l))
        {
            std::cerr << "Detected non-finite joint limits:" << std::endl;
            const size_t nlim = bounds.size() / 2;
            for (uint i = 0; i < nlim; ++i)
            {
                std::cout << bounds[i] << ", " << bounds[nlim + i] << std::endl;
            }
            throw std::runtime_error("All joint limits need to be finite!");
        }
    }

    if (!state_space_->as<OMPLStateSpace>()->isLocked())
    {
        state_space_->as<OMPLStateSpace>()->SetBounds(prob_);
        bounds_ = prob_->GetBounds();
    }
    else if (!bounds_.empty() && bounds_ != prob_->GetBounds())
    {
        ThrowPretty("Cannot set new bounds on locked state space!");
    }

    ompl_simple_setup_->getSpaceInformation()->setup();

    ompl_simple_setup_->setup();

    if (ompl_simple_setup_->getPlanner()->params().hasParam("Range"))
        ompl_simple_setup_->getPlanner()->params().setParam("Range", init_.Range);
    if (ompl_simple_setup_->getPlanner()->params().hasParam("GoalBias"))
        ompl_simple_setup_->getPlanner()->params().setParam("GoalBias", init_.GoalBias);

    if (init_.RandomSeed > -1)
    {
        HIGHLIGHT_NAMED(algorithm_, "Setting random seed to " << init_.RandomSeed);
        ompl::RNG::setSeed(static_cast<long unsigned int>(init_.RandomSeed));
    }

    SetGoalState(prob_->GetGoalState(), init_.Epsilon);

    ompl::base::ScopedState<> ompl_start_state(state_space_);

    state_space_->as<OMPLStateSpace>()->ExoticaToOMPLState(q0, ompl_start_state.get());
    ompl_simple_setup_->setStartState(ompl_start_state);

    PreSolve();
    ompl::time::point start = ompl::time::now();
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(init_.Timeout - ompl::time::seconds(ompl::time::now() - start));

    Timer t;
    if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION && ompl_simple_setup_->haveSolutionPath())
    {
        GetPath(solution, ptc);
    }
    planning_time_ = t.GetDuration();
    PostSolve();
}

template class OMPLSolver<SamplingProblem>;
}  // namespace exotica
