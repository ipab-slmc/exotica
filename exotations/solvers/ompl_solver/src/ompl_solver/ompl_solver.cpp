/*
 *  Created on: 10 Oct 2017
 *      Author: Yiming Yang
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

#include <ompl/util/RandomNumbers.h>
#include <ompl_solver/ompl_solver.h>

namespace exotica
{

template <class ProblemType>
OMPLsolver<ProblemType>::OMPLsolver() : multiQuery(false)
{
}

template <class ProblemType>
OMPLsolver<ProblemType>::~OMPLsolver()
{
}

template <class ProblemType>
void OMPLsolver<ProblemType>::specifyProblem(PlanningProblem_ptr pointer)

{
    MotionSolver::specifyProblem(pointer);
    prob_ = std::static_pointer_cast<ProblemType>(pointer);
    if (prob_->getScene()->getBaseType() == BASE_TYPE::FIXED)
        state_space_.reset(new OMPLRNStateSpace(prob_, init_));
    else if (prob_->getScene()->getBaseType() == BASE_TYPE::PLANAR)
        state_space_.reset(new OMPLSE2RNStateSpace(prob_, init_));
    else if (prob_->getScene()->getBaseType() == BASE_TYPE::FLOATING)
        state_space_.reset(new OMPLSE3RNStateSpace(prob_, init_));
    else
        throw_named("Unsupported base type " << prob_->getScene()->getBaseType());
    ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(state_space_));
    ompl_simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new OMPLStateValidityChecker(ompl_simple_setup_->getSpaceInformation(), prob_)));
    ompl_simple_setup_->setPlannerAllocator(boost::bind(planner_allocator_, _1, "Exotica_" + algorithm_));

    if (init_.Projection.rows() > 0)
    {
        std::vector<int> project_vars(init_.Projection.rows());
        for (int i = 0; i < init_.Projection.rows(); i++)
        {
            project_vars[i] = (int)init_.Projection(i);
            if (project_vars[i] < 0 || project_vars[i] >= prob_->N) throw_named("Invalid projection index! " << project_vars[i]);
        }
        if (prob_->getScene()->getBaseType() == BASE_TYPE::FIXED)
            ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLRNProjection(state_space_, project_vars)));
        else if (prob_->getScene()->getBaseType() == BASE_TYPE::PLANAR)
            ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLSE2RNProjection(state_space_, project_vars)));
        else if (prob_->getScene()->getBaseType() == BASE_TYPE::FLOATING)
            ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new OMPLSE3RNProjection(state_space_, project_vars)));
    }

    ompl_simple_setup_->getSpaceInformation()->setup();
    ompl_simple_setup_->setup();
    if (ompl_simple_setup_->getPlanner()->params().hasParam("range"))
        ompl_simple_setup_->getPlanner()->params().setParam("range", init_.Range);
    if (ompl_simple_setup_->getPlanner()->params().hasParam("goal_bias"))
        ompl_simple_setup_->getPlanner()->params().setParam("goal_bias", init_.GoalBias);

    if (init_.RandomSeed != -1)
    {
        HIGHLIGHT_NAMED(algorithm_, "Setting random seed to " << init_.RandomSeed);
        ompl::RNG::setSeed(init_.RandomSeed);
    }
}

template <class ProblemType>
int OMPLsolver<ProblemType>::getRandomSeed()
{
    return ompl::RNG::getSeed();
}

template <class ProblemType>
void OMPLsolver<ProblemType>::preSolve()
{
    // clear previously computed solutions
    if (!multiQuery)
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
void OMPLsolver<ProblemType>::postSolve()
{
    ompl_simple_setup_->clearStartStates();
    int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
    int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
    logDebug("There were %d valid motions and %d invalid motions.", v, iv);

    if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
        logWarn("Computed solution is approximate");
}

template <class ProblemType>
void OMPLsolver<ProblemType>::setGoalState(const Eigen::VectorXd &qT, const double eps)
{
    ompl::base::ScopedState<> gs(state_space_);
    state_space_->as<OMPLStateSpace>()->ExoticaToOMPLState(qT, gs.get());
    if (!ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
    {
        throw_named("Goal state is not valid!");
    }

    if (!ompl_simple_setup_->getSpaceInformation()->satisfiesBounds(gs.get()))
    {
        state_space_->as<OMPLStateSpace>()->stateDebug(qT);

        // Debug state and bounds
        std::string out_of_bounds_joint_ids = "";
        for (int i = 0; i < qT.rows(); i++)
            if (qT(i) < prob_->getBounds()[i] || qT(i) > prob_->getBounds()[i + qT.rows()])
                out_of_bounds_joint_ids += "[j" + std::to_string(i) + "=" + std::to_string(qT(i)) + ", ll=" + std::to_string(prob_->getBounds()[i]) + ", ul=" + std::to_string(prob_->getBounds()[i + qT.rows()]) + "]\n";

        throw_named("Invalid goal state [Invalid joint bounds for joint indices: \n"
                    << out_of_bounds_joint_ids << "]");
    }
    ompl_simple_setup_->setGoalState(gs, eps);
}

template <class ProblemType>
void OMPLsolver<ProblemType>::getPath(Eigen::MatrixXd &traj, ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::geometric::PathSimplifierPtr psf_ = ompl_simple_setup_->getPathSimplifier();
    const ompl::base::SpaceInformationPtr &si = ompl_simple_setup_->getSpaceInformation();

    ompl::geometric::PathGeometric pg = ompl_simple_setup_->getSolutionPath();
    if (init_.Smooth)
    {
        bool tryMore = false;
        if (ptc == false)
            tryMore = psf_->reduceVertices(pg);
        if (ptc == false)
            psf_->collapseCloseVertices(pg);
        int times = 0;
        while (times < 10 && tryMore && ptc == false)
        {
            tryMore = psf_->reduceVertices(pg);
            times++;
        }
        if (si->getStateSpace()->isMetricSpace())
        {
            if (ptc == false)
                tryMore = psf_->shortcutPath(pg);
            else
                tryMore = false;
            while (times < 10 && tryMore && ptc == false)
            {
                tryMore = psf_->shortcutPath(pg);
                times++;
            }
        }
    }
    std::vector<ompl::base::State *> &states = pg.getStates();
    unsigned int length = 0;
    const int n1 = states.size() - 1;
    for (int i = 0; i < n1; ++i)
        length += si->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
    pg.interpolate(int(length * init_.SmoothnessFactor));

    traj.resize(pg.getStateCount(), prob_->getSpaceDim());
    Eigen::VectorXd tmp(prob_->getSpaceDim());

    for (int i = 0; i < (int)pg.getStateCount(); ++i)
    {
        state_space_->as<OMPLStateSpace>()->OMPLToExoticaState(pg.getState(i), tmp);
        traj.row(i) = tmp;
    }
}

template <class ProblemType>
void OMPLsolver<ProblemType>::Solve(Eigen::MatrixXd &solution)
{
    Eigen::VectorXd q0 = prob_->applyStartState();
    setGoalState(prob_->goal_, init_.Epsilon);

    ompl::base::ScopedState<> ompl_start_state(state_space_);
    state_space_->as<OMPLStateSpace>()->ExoticaToOMPLState(q0, ompl_start_state.get());
    ompl_simple_setup_->setStartState(ompl_start_state);

    preSolve();
    ompl::time::point start = ompl::time::now();
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(init_.Timeout - ompl::time::seconds(ompl::time::now() - start));
    if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION && ompl_simple_setup_->haveSolutionPath())
    {
        getPath(solution, ptc);
    }
    postSolve();
}

template class OMPLsolver<SamplingProblem>;

}
