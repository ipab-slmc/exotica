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
    getSubspace(1)->as<ompl::base::TimeStateSpace>()->setBounds(0, prob_->T);
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
    if (q.rows() != prob_->N) q.resize(prob_->N);
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
    planner_allocator_ = boost::bind(&allocatePlanner<OMPLTimeIndexedRRTConnect>, _1, _2);
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
    ompl_simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(init_.ValidityCheckResolution);

    ompl_simple_setup_->getSpaceInformation()->setup();
    ompl_simple_setup_->setup();
    if (ompl_simple_setup_->getPlanner()->params().hasParam("range")) ompl_simple_setup_->getPlanner()->params().setParam("range", init_.Range);
}

void TimeIndexedRRTConnect::preSolve()
{
    // clear previously computed solutions
    ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
    const ompl::base::PlannerPtr planner = ompl_simple_setup_->getPlanner();
    if (planner) planner->clear();
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
            if (qT(i) < prob_->getBounds()[i] || qT(i) > prob_->getBounds()[i + qT.rows()]) out_of_bounds_joint_ids += "[j" + std::to_string(i) + "=" + std::to_string(qT(i)) + ", ll=" + std::to_string(prob_->getBounds()[i]) + ", ul=" + std::to_string(prob_->getBounds()[i + qT.rows()]) + "]\n";

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
        if (ptc == false) tryMore = psf_->reduceVertices(pg);
        if (ptc == false) psf_->collapseCloseVertices(pg);
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

    traj.resize(pg.getStateCount(), init_.AddTimeIntoSolution ? prob_->getSpaceDim() + 1 : prob_->getSpaceDim());
    Eigen::VectorXd tmp(prob_->getSpaceDim());
    Eigen::VectorXd ts(pg.getStateCount());
    for (int i = 0; i < (int)pg.getStateCount(); ++i)
    {
        state_space_->as<OMPLTimeIndexedRNStateSpace>()->OMPLToExoticaState(pg.getState(i), tmp, ts(i));
        traj.row(i).tail(prob_->getSpaceDim()) = tmp;
    }
    if (init_.AddTimeIntoSolution) traj.col(0) = ts;
}

void TimeIndexedRRTConnect::Solve(Eigen::MatrixXd &solution)
{
    prob_->preupdate();
    Eigen::VectorXd q0 = prob_->applyStartState();
    setGoalState(prob_->goal_, prob_->tGoal);

    ompl::base::ScopedState<> ompl_start_state(state_space_);
    state_space_->as<OMPLTimeIndexedRNStateSpace>()->ExoticaToOMPLState(q0, prob_->tStart, ompl_start_state.get());
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

OMPLTimeIndexedRRTConnect::OMPLTimeIndexedRRTConnect(const base::SpaceInformationPtr &si) : base::Planner(si, "OMPLTimeIndexedRRTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    maxDistance_ = 0.0;

    Planner::declareParam<double>("range", this, &OMPLTimeIndexedRRTConnect::setRange, &OMPLTimeIndexedRRTConnect::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State *, base::State *>(NULL, NULL);
}

OMPLTimeIndexedRRTConnect::~OMPLTimeIndexedRRTConnect()
{
    freeMemory();
}

void OMPLTimeIndexedRRTConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

#ifdef ROS_KINETIC
    if (!tStart_) tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_) tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
#else
    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(si_->getStateSpace()));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(si_->getStateSpace()));
#endif
    tStart_->setDistanceFunction(boost::bind(&OMPLTimeIndexedRRTConnect::reverseTimeDistance, this, _1, _2));
    tGoal_->setDistanceFunction(boost::bind(&OMPLTimeIndexedRRTConnect::forwardTimeDistance, this, _1, _2));
}

void OMPLTimeIndexedRRTConnect::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i)
        {
            if (motions[i]->state) si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i)
        {
            if (motions[i]->state) si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void OMPLTimeIndexedRRTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_) tStart_->clear();
    if (tGoal_) tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(NULL, NULL);
}

OMPLTimeIndexedRRTConnect::GrowState OMPLTimeIndexedRRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    bool changed = false;
    if (!correctTime(nmotion, rmotion, !tgi.start, changed)) return TRAPPED;

    /* assume we can reach the state we go towards */
    bool reach = !changed;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
    // so we check that one first
    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) : si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (validMotion)
    {
        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        tree->add(motion);
        if (reach)
        {
            return REACHED;
        }
        else
        {
            return ADVANCED;
        }
    }
    else
        return TRAPPED;
}

ompl::base::PlannerStatus OMPLTimeIndexedRRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    std::cout << "getStartStateCount() in OMPLTimeIndexedRRTConnect: " << pdef_->getStartStateCount() << std::endl;
    const base::State *check_st = pdef_->getStartState(0);
    std::cout << "isValid: in OMPLTimeIndexedRRTConnect: " << si_->isValid(check_st) << std::endl;

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_) sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    while (ptc == false)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);
        reverse_check_ = false;
        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED) si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;

            reverse_check_ = true;
            while (ptc == false && gsc == ADVANCED)
            {
                gsc = growTree(otherTree, tgi, rmotion);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not NULL. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != NULL)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != NULL)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (unsigned int i = 0; i < mpath2.size(); ++i)
                    path->append(mpath2[i]->state);

                pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
                solved = true;
                break;
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void OMPLTimeIndexedRRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_) tStart_->list(motions);

    for (unsigned int i = 0; i < motions.size(); ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1), base::PlannerDataVertex(motions[i]->state, 1));
        }
    }

    motions.clear();
    if (tGoal_) tGoal_->list(motions);

    for (unsigned int i = 0; i < motions.size(); ++i)
    {
        if (motions[i]->parent == NULL)
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state, 2), base::PlannerDataVertex(motions[i]->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
}
