/*
 * frrt.cpp
 *
 *  Created on: 22 Apr 2015
 *      Author: yiming
 */

#include "frrt/FRRT.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"

namespace ompl
{
	namespace geometric
	{
		FRRT::FRRT(const base::SpaceInformationPtr &si) :
				base::Planner(si, "FRRT")
		{
			specs_.approximateSolutions = true;
			specs_.directed = true;
			goalBias_ = 0.05;
			maxDist_ = 0.0;
			lastGoalMotion_ = NULL;
			Planner::declareParam<double>("range", this, &FRRT::setRange, &FRRT::getRange, "0.:1.:10000.");
			Planner::declareParam<double>("goal_bias", this, &FRRT::setGoalBias, &FRRT::getGoalBias, "0.:.05:1.");
		}

		FRRT::~FRRT()
		{
			freeMemory();
		}

		void FRRT::clear()
		{
			Planner::clear();
			sampler_.reset();
			freeMemory();
			if (nn_)
				nn_->clear();
			lastGoalMotion_ = NULL;
		}

		void FRRT::setup()
		{
			Planner::setup();
			tools::SelfConfig sc(si_, getName());
			sc.configurePlannerRange(maxDist_);

			if (!nn_)
				nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
			nn_->setDistanceFunction(boost::bind(&FRRT::distanceFunction, this, _1, _2));
		}

		void FRRT::freeMemory()
		{
			std::vector<Motion*> motions;
			nn_->list(motions);
			for (unsigned int i = 0; i < motions.size(); ++i)
			{
				if (motions[i]->state)
					si_->freeState(motions[i]->state);
				delete motions[i];
			}
		}

		base::PlannerStatus FRRT::solve(const base::PlannerTerminationCondition &ptc)
		{
			if (!local_solver_)
			{
				INDICATE_FAILURE
				return base::PlannerStatus::CRASH;
			}
			checkValidity();
			base::Goal *goal = pdef_->getGoal().get();
			base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
			while (const base::State *st = pis_.nextStart())
			{
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				nn_->add(motion);
			}

			if (nn_->size() == 0)
			{
				OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
				INDICATE_FAILURE
				return base::PlannerStatus::INVALID_START;
			}
			if (!sampler_)
				sampler_ = si_->allocStateSampler();
			OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

			Motion *solution = NULL;
			Motion *approxsol = NULL;
			double approxdif = std::numeric_limits<double>::infinity();
			Motion *rmotion = new Motion(si_);
			base::State *rstate = rmotion->state;
			base::State *xstate = si_->allocState();
			while (ptc == false)
			{
				/* sample random state (with goal biasing) */
				if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
					goal_s->sampleGoal(rstate);
				else
					sampler_->sampleUniform(rstate);

				/* find closest state in the tree */
				Motion *nmotion = nn_->nearest(rmotion);
				base::State *dstate = rstate;

				//	For FRRT, we dont need this
				/* find state to add */
//				double d = si_->distance(nmotion->state, rstate);
//				if (d > maxDist_)
//				{
//					si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDist_ / d, xstate);
//					dstate = xstate;
//				}
//				std::pair<ompl::base::State*, double> last_state_;
				Motion *motion = new Motion(si_);
				bool valid_state = false;
				if (si_->checkMotion(nmotion->state, dstate))
				{
					si_->copyState(motion->state, dstate);
					motion->parent = nmotion;
					valid_state = true;
				}
				else
				{
					/* This is where the local planner goes */
					int dim = (int) si_->getStateDimension();
					ERROR("Dim="<<dim);
					Eigen::VectorXd qs(dim), qg(dim);
					INDICATE_FAILURE
					memcpy(qs.data(), nmotion->state->as<
							ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
							* qs.rows());
					INDICATE_FAILURE
					memcpy(qg.data(), dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
							* qg.rows());
					ERROR("q0="<<qs.transpose());
					ERROR("qg="<<qg.transpose());

					if (!ok(local_solver_->setGoal("CSpaceTask", qg)))
					{
						INDICATE_FAILURE
						return base::PlannerStatus::INVALID_GOAL;
					}
					INDICATE_FAILURE
					Eigen::MatrixXd local_path;
					if (ok(local_solver_->Solve(qs, local_path)))
					{
						/* Local planner succeeded */
						ERROR("Find local path");
						si_->copyState(motion->state, dstate);
						motion->parent = nmotion;
						*motion->interal_path = local_path;
						valid_state = true;
					}

				}
				if (valid_state)
				{
					nn_->add(motion);
					double dist = 0.0;
					bool sat = goal->isSatisfied(motion->state, &dist);
					if (sat)
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
					Motion *local_motion;
					if (solution->interal_path != nullptr)
					{
						for (int i = 0; i < solution->interal_path->rows(); i++)
						{
							Eigen::VectorXd tmp = solution->interal_path->row(i).transpose();
							memcpy(local_motion->state->as<
									ompl::base::RealVectorStateSpace::StateType>()->values, tmp.data(), sizeof(double)
									* tmp.rows());
							mpath.push_back(local_motion);
						}
					}
					solution = solution->parent;
				}
				PathGeometric *path = new PathGeometric(si_);
				for (int i = mpath.size() - 1; i >= 0; --i)
					path->append(mpath[i]->state);
				pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
				solved = true;
			}

			si_->freeState(xstate);
			if (rmotion->state)
				si_->freeState(rmotion->state);
			delete rmotion;
			OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
			return base::PlannerStatus(solved, approximate);
		}

		void FRRT::getPlannerData(base::PlannerData &data) const
		{
			Planner::getPlannerData(data);
			std::vector<Motion*> motions;
			if (nn_)
				nn_->list(motions);
			if (lastGoalMotion_)
				data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
			for (unsigned int i = 0; i < motions.size(); ++i)
			{
				if (motions[i]->parent == NULL)
					data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
				else
					data.addEdge(base::PlannerDataVertex(motions[i]->parent->state), base::PlannerDataVertex(motions[i]->state));
			}
		}

		bool FRRT::setUpLocalPlanner(const std::string & xml_file, const exotica::Scene_ptr & scene)
		{
			exotica::Initialiser ini;
			exotica::Server_ptr ser;
			exotica::PlanningProblem_ptr prob;
			exotica::MotionSolver_ptr sol;
			if (!ok(ini.initialise(xml_file, ser, sol, prob)))
			{
				INDICATE_FAILURE
				return false;
			}
			if (sol->type().compare("exotica::IKsolver") == 0)
			{
				local_solver_ = boost::static_pointer_cast<exotica::IKsolver>(sol);
			}
			else
			{
				INDICATE_FAILURE
				return false;
			}
			if (!ok(local_solver_->specifyProblem(prob)))
			{
				INDICATE_FAILURE
				return false;
			}
			prob->setScene(scene->getPlanningScene());
			return true;
		}
	}	//	Namespace geometric
}	//	Namespace ompl

