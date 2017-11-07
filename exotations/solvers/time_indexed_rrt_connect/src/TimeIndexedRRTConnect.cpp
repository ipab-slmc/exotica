/*
 *  Created on: 7 Nov 2017
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

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <time_indexed_rrt_connect/TimeIndexedRRTConnect.h>

REGISTER_MOTIONSOLVER_TYPE("TimeIndexedRRTConnect", exotica::TimeIndexedRRTConnect)

namespace exotica
{
OMPLTimeIndexedRNStateSpace::OMPLTimeIndexedRNStateSpace(TimeIndexedSamplingProblem_ptr &prob, TimeIndexedRRTConnectInitializer init) : ompl::base::CompoundStateSpace(), prob_(prob)
{
    setName("OMPLTimeIndexedRNStateSpace");
    unsigned int dim = prob->N;
    addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim)), 1.0);
    ompl::base::RealVectorBounds bounds(dim);
    for (int i = 0; i < dim; i++)
    {
        bounds.setHigh(i, prob->getBounds()[i + dim]);
        bounds.setLow(i, prob->getBounds()[i]);
    }
    getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    addSubspace(ompl::base::StateSpacePtr(new ompl::base::TimeStateSpace), 1.0);
    getSubspace(1)->as<ompl::base::TimeStateSpace>()->setBounds(0, prob_->T_);
    lock();
}

ompl::base::StateSamplerPtr OMPLTimeIndexedRNStateSpace::allocDefaultStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}
void OMPLTimeIndexedRNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q, const double &t, ompl::base::State *state) const
{
    OMPLTimeIndexedRNStateSpace::StateType *ss = static_cast<OMPLTimeIndexedRNStateSpace::StateType *>(state);
    memcpy(ss->getRNSpace().values, q.data(), sizeof(double) * q.rows());
    ss->getTime().position = t;
}
void OMPLTimeIndexedRNStateSpace::OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q, double &t) const
{
    const OMPLTimeIndexedRNStateSpace::StateType *ss = static_cast<const OMPLTimeIndexedRNStateSpace::StateType *>(state);
    memcpy(q.data(), ss->getRNSpace().values, sizeof(double) * prob_->N);
    t = ss->getTime().position;
}
void OMPLTimeIndexedRNStateSpace::stateDebug(const Eigen::VectorXd &q) const
{
}

OMPLTimeIndexedStateValidityChecker::OMPLTimeIndexedStateValidityChecker(const ompl::base::SpaceInformationPtr &si, const TimeIndexedSamplingProblem_ptr &prob) : ompl::base::StateValidityChecker(si), prob_(prob)
{
}

bool OMPLTimeIndexedStateValidityChecker::isValid(const ompl::base::State *state) const
{
    double tmp;
    return isValid(state, tmp);
}

bool OMPLTimeIndexedStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
{
    Eigen::VectorXd q(prob_->N);
    double t;
#ifdef ROS_KINETIC
    std::static_pointer_cast<OMPLTimeIndexedRNStateSpace>(si_->getStateSpace())->OMPLToExoticaState(state, q, t);
#else
    boost::static_pointer_cast<OMPLTimeIndexedRNStateSpace>(si_->getStateSpace())->OMPLToExoticaState(state, q, t);
#endif

    if (!prob_->isValid(q, t))
    {
        dist = -1;
        return false;
    }
    return true;
}

// The solver
TimeIndexedRRTConnect::TimeIndexedRRTConnect()
{
}

TimeIndexedRRTConnect::~TimeIndexedRRTConnect()
{
}

void TimeIndexedRRTConnect::Instantiate(TimeIndexedRRTConnectInitializer &init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_TimeIndexedRRTConnect";
    planner_allocator_ = boost::bind(&allocatePlanner<ompl::geometric::RRTConnect>, _1, _2);
}

void TimeIndexedRRTConnect::specifyProblem(PlanningProblem_ptr pointer)

{
    MotionSolver::specifyProblem(pointer);
    prob_ = std::static_pointer_cast<TimeIndexedSamplingProblem>(pointer);
    if (prob_->getScene()->getBaseType() == BASE_TYPE::FIXED)
        state_space_.reset(new OMPLTimeIndexedRNStateSpace(prob_, init_));
    else
        throw_named("Unsupported base type " << prob_->getScene()->getBaseType() << ", the time indexed solver can only solve for fixed base robots");
    ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(state_space_));
    ompl_simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new OMPLTimeIndexedStateValidityChecker(ompl_simple_setup_->getSpaceInformation(), prob_)));
    ompl_simple_setup_->setPlannerAllocator(boost::bind(planner_allocator_, _1, "Exotica_" + algorithm_));

    ompl_simple_setup_->getSpaceInformation()->setup();
    ompl_simple_setup_->setup();
    if (ompl_simple_setup_->getPlanner()->params().hasParam("range"))
        ompl_simple_setup_->getPlanner()->params().setParam("range", init_.Range);
}

void TimeIndexedRRTConnect::preSolve()
{
    // clear previously computed solutions
    ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
    const ompl::base::PlannerPtr planner = ompl_simple_setup_->getPlanner();
    if (planner)
        planner->clear();
    ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
    ompl_simple_setup_->getPlanner()->setProblemDefinition(ompl_simple_setup_->getProblemDefinition());
}

void TimeIndexedRRTConnect::postSolve()
{
    ompl_simple_setup_->clearStartStates();
    int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
    int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
    logDebug("There were %d valid motions and %d invalid motions.", v, iv);

    if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
        logWarn("Computed solution is approximate");
}

void TimeIndexedRRTConnect::setGoalState(const Eigen::VectorXd &qT, const double t, const double eps)
{
    ompl::base::ScopedState<> gs(state_space_);
    state_space_->as<OMPLTimeIndexedRNStateSpace>()->ExoticaToOMPLState(qT, t, gs.get());
    if (!ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
    {
        throw_named("Goal state is not valid!");
    }

    if (!ompl_simple_setup_->getSpaceInformation()->satisfiesBounds(gs.get()))
    {
        state_space_->as<OMPLTimeIndexedRNStateSpace>()->stateDebug(qT);

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

void TimeIndexedRRTConnect::getPath(Eigen::MatrixXd &traj, ompl::base::PlannerTerminationCondition &ptc)
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
    pg.interpolate(length);

    traj.resize(pg.getStateCount(), prob_->getSpaceDim());
    Eigen::VectorXd tmp(prob_->getSpaceDim());
    Eigen::VectorXd ts(pg.getStateCount());
    for (int i = 0; i < (int)pg.getStateCount(); ++i)
    {
        state_space_->as<OMPLTimeIndexedRNStateSpace>()->OMPLToExoticaState(pg.getState(i), tmp, ts(i));
        traj.row(i) = tmp;
    }
}

void TimeIndexedRRTConnect::Solve(Eigen::MatrixXd &solution)
{
    prob_->preupdate();
    Eigen::VectorXd q0 = prob_->applyStartState();
    setGoalState(prob_->goal_, prob_->T_);

    ompl::base::ScopedState<> ompl_start_state(state_space_);
    state_space_->as<OMPLTimeIndexedRNStateSpace>()->ExoticaToOMPLState(q0, 0, ompl_start_state.get());
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
}
