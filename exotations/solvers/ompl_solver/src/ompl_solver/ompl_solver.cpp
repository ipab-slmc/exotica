/*
 * ompl_solver.cpp
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#include <ompl_solver/ompl_solver.h>

namespace exotica
{
OMPLsolver::OMPLsolver()
{
}

OMPLsolver::~OMPLsolver()
{
}

void OMPLsolver::specifyProblem(PlanningProblem_ptr pointer)

{
    MotionSolver::specifyProblem(pointer);
    prob_ = std::static_pointer_cast<SamplingProblem>(pointer);

    state_space_.reset(new OMPLRNStateSpace(prob_->N, prob_, init_));

    ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(state_space_));
    ompl_simple_setup_->setStateValidityChecker(
        ompl::base::StateValidityCheckerPtr(
            new OMPLStateValidityChecker(
                ompl_simple_setup_->getSpaceInformation(), prob_)));
    ompl_simple_setup_->setPlannerAllocator(
        boost::bind(planner_allocator_, _1, "Exotica_" + algorithm_));

    ompl_simple_setup_->getSpaceInformation()->setup();
    ompl_simple_setup_->setup();
}

void OMPLsolver::preSolve()
{
    // clear previously computed solutions
    ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
    const ompl::base::PlannerPtr planner = ompl_simple_setup_->getPlanner();
    if (planner)
        planner->clear();
    ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
    ompl_simple_setup_->getPlanner()->setProblemDefinition(
        ompl_simple_setup_->getProblemDefinition());
}

void OMPLsolver::postSolve()
{
    ompl_simple_setup_->clearStartStates();
    int v =
        ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
    int iv =
        ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
    logDebug("There were %d valid motions and %d invalid motions.", v, iv);

    if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
        logWarn("Computed solution is approximate");
}

void OMPLsolver::setGoalState(const Eigen::VectorXd &qT, const double eps)
{
    ompl::base::ScopedState<> gs(state_space_);
    state_space_->as<OMPLBaseStateSpace>()->ExoticaToOMPLState(qT, gs.get());
    if (!ompl_simple_setup_->getStateValidityChecker()->isValid(gs.get()))
    {
        throw_named("Goal state is not valid!");
    }

    if (!ompl_simple_setup_->getSpaceInformation()->satisfiesBounds(gs.get()))
    {
        state_space_->as<OMPLBaseStateSpace>()->stateDebug(qT);

        // Debug state and bounds
        std::string out_of_bounds_joint_ids = "";
        for (int i = 0; i < qT.rows(); i++)
            if (qT(i) < prob_->getBounds()[i] || qT(i) > prob_->getBounds()[i + qT.rows()])
                out_of_bounds_joint_ids += "[j" + std::to_string(i) + "=" + std::to_string(qT(i)) + ", ll=" + std::to_string(prob_->getBounds()[i]) + ", ul=" + std::to_string(prob_->getBounds()[i + qT.rows()]) + "]\n";

        throw_named(
            "Invalid goal state [Invalid joint bounds for joint indices: \n"
            << out_of_bounds_joint_ids << "]");
    }
    ompl_simple_setup_->setGoalState(gs, eps);
}

void OMPLsolver::getPath(Eigen::MatrixXd &traj,
                         ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::geometric::PathSimplifierPtr psf_ =
        ompl_simple_setup_->getPathSimplifier();
    const ompl::base::SpaceInformationPtr &si =
        ompl_simple_setup_->getSpaceInformation();

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
        length += si->getStateSpace()->validSegmentCount(states[i],
                                                         states[i + 1]);
    pg.interpolate(length);

    traj.resize(pg.getStateCount(), prob_->getSpaceDim());
    Eigen::VectorXd tmp(prob_->getSpaceDim());

    for (int i = 0; i < (int)pg.getStateCount(); ++i)
    {
        state_space_->as<OMPLBaseStateSpace>()->OMPLToExoticaState(
            pg.getState(i), tmp);
        traj.row(i) = tmp;
    }
}

void OMPLsolver::Solve(Eigen::MatrixXd &solution)
{
    prob_->preupdate();
    Eigen::VectorXd q0 = prob_->applyStartState();
    setGoalState(prob_->goal_);

    ompl::base::ScopedState<> ompl_start_state(state_space_);
    state_space_->as<OMPLBaseStateSpace>()->ExoticaToOMPLState(q0,
                                                               ompl_start_state.get());
    ompl_simple_setup_->setStartState(ompl_start_state);

    preSolve();
    ompl::time::point start = ompl::time::now();
    ompl::base::PlannerTerminationCondition ptc =
        ompl::base::timedPlannerTerminationCondition(
            init_.Timeout - ompl::time::seconds(ompl::time::now() - start));
    if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION && ompl_simple_setup_->haveSolutionPath())
    {
        getPath(solution, ptc);
    }
    postSolve();
}
}
