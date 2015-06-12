/*
 * FKPIECE.cpp
 *
 *  Created on: 12 Jun 2015
 *      Author: yiming
 */

#include "frrt/FKPIECE.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::FKPIECE::FKPIECE(const base::SpaceInformationPtr &si) :
		FlexiblePlanner(si, "FKPIECE"), disc_(boost::bind(&FKPIECE::freeMotion, this, _1))
{
	goalBias_ = 0.05;
	failedExpansionScoreFactor_ = 0.5;
	minValidPathFraction_ = 0.2;
	maxDistance_ = 0.0;
	lastGoalMotion_ = NULL;

	Planner::declareParam<double>("range", this, &FKPIECE::setRange, &FKPIECE::getRange, "0.:1.:10000.");
	Planner::declareParam<double>("goal_bias", this, &FKPIECE::setGoalBias, &FKPIECE::getGoalBias, "0.:.05:1.");
	Planner::declareParam<double>("border_fraction", this, &FKPIECE::setBorderFraction, &FKPIECE::getBorderFraction, "0.:0.05:1.");
	Planner::declareParam<double>("failed_expansion_score_factor", this, &FKPIECE::setFailedExpansionCellScoreFactor, &FKPIECE::getFailedExpansionCellScoreFactor);
	Planner::declareParam<double>("min_valid_path_fraction", this, &FKPIECE::setMinValidPathFraction, &FKPIECE::getMinValidPathFraction);
}

ompl::geometric::FKPIECE::~FKPIECE()
{
}

void ompl::geometric::FKPIECE::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configureProjectionEvaluator(projectionEvaluator_);
	sc.configurePlannerRange(maxDistance_);

	if (failedExpansionScoreFactor_ < std::numeric_limits<double>::epsilon()
			|| failedExpansionScoreFactor_ > 1.0)
		throw Exception("Failed expansion cell score factor must be in the range (0,1]");
	if (minValidPathFraction_ < std::numeric_limits<double>::epsilon()
			|| minValidPathFraction_ > 1.0)
		throw Exception("The minimum valid path fraction must be in the range (0,1]");

	disc_.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::FKPIECE::clear()
{
	Planner::clear();
	sampler_.reset();
	disc_.clear();
	lastGoalMotion_ = NULL;
}

void ompl::geometric::FKPIECE::freeMotion(Motion *motion)
{
	if (motion->state)
		si_->freeState(motion->state);
	delete motion;
}

ompl::base::PlannerStatus ompl::geometric::FKPIECE::solve(
		const base::PlannerTerminationCondition &ptc)
{
	checkValidity();
	base::Goal *goal = pdef_->getGoal().get();
	base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	Discretization<Motion>::Coord xcoord;

	while (const base::State *st = pis_.nextStart())
	{
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		projectionEvaluator_->computeCoordinates(motion->state, xcoord);
		disc_.addMotion(motion, xcoord, 1.0);
	}

	if (disc_.getMotionCount() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), disc_.getMotionCount());

	Motion *solution = NULL;
	Motion *approxsol = NULL;
	double approxdif = std::numeric_limits<double>::infinity();
	base::State *xstate = si_->allocState();

	while (ptc == false)
	{
		disc_.countIteration();

		/* Decide on a state to expand from */
		Motion *existing = NULL;
		Discretization<Motion>::Cell *ecell = NULL;
		disc_.selectMotion(existing, ecell);
		assert(existing);

		/* sample random state (with goal biasing) */
		if (goal_s && goal_s->canSample())
			goal_s->sampleGoal(xstate);
		{
			ompl::base::State *last_valid_state = si_->allocState();
			std::pair<ompl::base::State*, double> last_valid(last_valid_state, 0);
			bool keep = si_->checkMotion(existing->state, xstate, last_valid);
			if (!keep && last_valid.second > minValidPathFraction_)
			{
				memcpy(local_map_->jointRef.data(), xstate->as<
						ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
						* local_map_->jointRef.rows());
				local_solver_->getProblem()->setTau(gTau_->data);
				Motion * new_motion = new Motion(si_);
				bool solved = localSolve(existing, last_valid_state, new_motion);
				if (new_motion->internal_path)
					keep = true;

				if (solved)
				{
					double dist = 0.0;
					bool solv = goal->isSatisfied(new_motion->state, &dist);
					projectionEvaluator_->computeCoordinates(new_motion->state, xcoord);
					disc_.addMotion(new_motion, xcoord, dist); // this will also update the discretization heaps as needed, so no call to updateCell() is needed
					if (solv)
					{
						approxdif = dist;
						solution = new_motion;
						break;
					}
					if (dist < approxdif)
					{
						approxdif = dist;
						approxsol = new_motion;
					}
				}

			}
			else if (keep)
			{
				/* create a motion */
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, xstate);
				motion->parent = existing;

				double dist = 0.0;
				bool solv = goal->isSatisfied(motion->state, &dist);
				projectionEvaluator_->computeCoordinates(motion->state, xcoord);
				disc_.addMotion(motion, xcoord, dist); // this will also update the discretization heaps as needed, so no call to updateCell() is needed

				if (solv)
				{
					approxdif = dist;
					solution = motion;
					break;
				}
				if (dist < approxdif)
				{
					approxdif = dist;
					approxsol = motion;
				}
			}
			else
				ecell->data->score *= failedExpansionScoreFactor_;
		}

		do
		{
			sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);
		}
		while (!si_->getStateValidityChecker()->isValid(xstate) && ptc == false);
		ompl::base::State *last_valid_state = si_->allocState();
		std::pair<ompl::base::State*, double> last_valid(last_valid_state, 0);
		bool keep = si_->checkMotion(existing->state, xstate, last_valid);
		if (!keep && last_valid.second > minValidPathFraction_)
		{
			memcpy(local_map_->jointRef.data(), xstate->as<
					ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
					* local_map_->jointRef.rows());
			local_solver_->getProblem()->setTau(gTau_->data);
			Motion * new_motion = new Motion(si_);
			bool solved = localSolve(existing, last_valid_state, new_motion);
			if (new_motion->internal_path)
				keep = true;

			if (solved)
			{
				double dist = 0.0;
				bool solv = goal->isSatisfied(new_motion->state, &dist);
				projectionEvaluator_->computeCoordinates(new_motion->state, xcoord);
				disc_.addMotion(new_motion, xcoord, dist); // this will also update the discretization heaps as needed, so no call to updateCell() is needed
				if (solv)
				{
					approxdif = dist;
					solution = new_motion;
					break;
				}
				if (dist < approxdif)
				{
					approxdif = dist;
					approxsol = new_motion;
				}
			}

		}
		else if (keep)
		{
			/* create a motion */
			Motion *motion = new Motion(si_);
			si_->copyState(motion->state, xstate);
			motion->parent = existing;

			double dist = 0.0;
			bool solv = goal->isSatisfied(motion->state, &dist);
			projectionEvaluator_->computeCoordinates(motion->state, xcoord);
			disc_.addMotion(motion, xcoord, dist); // this will also update the discretization heaps as needed, so no call to updateCell() is needed

			if (solv)
			{
				approxdif = dist;
				solution = motion;
				break;
			}
			if (dist < approxdif)
			{
				approxdif = dist;
				approxsol = motion;
			}
		}
		else
			ecell->data->score *= failedExpansionScoreFactor_;

		disc_.updateCell(ecell);
	}

	bool solved = false;
	bool approximate = false;
	if (solution == NULL)
	{
		solution = approxsol;
		approximate = true;
	}

	if (solution != NULL)
	{
		lastGoalMotion_ = solution;

		/* construct the solution path */
		std::vector<Motion*> mpath;
		while (solution != NULL)
		{
			mpath.push_back(solution);
			solution = solution->parent;
		}

		/* set the solution path */
		PathGeometric *path = new PathGeometric(si_);
		for (int i = mpath.size() - 1; i >= 0; --i)
			path->append(mpath[i]->state);
		pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
		solved = true;
	}

	si_->freeState(xstate);

	OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(), disc_.getMotionCount(), disc_.getCellCount(), disc_.getGrid().countInternal(), disc_.getGrid().countExternal());

	return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::FKPIECE::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);
	disc_.getPlannerData(data, 0, true, lastGoalMotion_);

}

bool ompl::geometric::FKPIECE::localSolve(Motion *sm, base::State *is, Motion *gm)
{
	int dim = (int) si_->getStateDimension();
	Eigen::VectorXd qs(dim), qg(dim);
	memcpy(qs.data(),
			is == NULL ? sm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values : is->as<
					ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
			* qs.rows());
	Eigen::MatrixXd local_path;
	exotica::EReturn ret = FlexiblePlanner::localSolve(qs, qg, local_path);
	if (ok(ret))
	{
		/* Local planner succeeded */
		if (is)
		{
			gm->inter_state = si_->allocState();
			si_->copyState(gm->inter_state, is);
		}
		gm->internal_path.reset(new Eigen::MatrixXd(local_path));
		gm->parent = sm;
		qg = local_path.row(local_path.rows() - 1).transpose();
		memcpy(gm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values, qg.data(), sizeof(double)
				* qg.rows());
	}
	return ret == exotica::SUCCESS ? true : false;
}
