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

			try_cnt_.resize(3);
			suc_cnt_.resize(3);
			init_rho_.resize(3);
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
			for (int i = 0; i < 3; i++)
				try_cnt_[i] = suc_cnt_[i] = 0;
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
			nn_->clear();
			checkValidity();
			base::Goal *goal = pdef_->getGoal().get();
			base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
			Motion *init_motion = NULL;
			while (const base::State *st = pis_.nextStart())
			{
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				if (init_motion == NULL)
				{
					init_motion = new Motion(si_);
					si_->copyState(init_motion->state, st);
				}
				nn_->add(motion);
			}
			INFO_NAMED("FRRT","Checking if goal is valid");
			if (goal_s && goal_s->canSample())
			{
				Motion *goal_motion = new Motion(si_);
				goal_s->sampleGoal(goal_motion->state);

				if (!goal->isSatisfied(goal_motion->state))
				{
					ERROR("FRRT, Invalid goal state");
					return base::PlannerStatus::INVALID_GOAL;
				}
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

			///	Solution info
			Motion *solution = NULL;
			bool solved = false;
			for (int i = 0; i < 3; i++)
				try_cnt_[i] = suc_cnt_[i] = 0;

			///Lets do it clean and nice !!!!!!!
			bool newTry = true;
			Motion *start_motion = init_motion;
			while (ptc == false)
			{
				/// Move from start to goal
				Motion *new_motion = new Motion(si_);
				if (newTry)
				{
					newTry = false;
					try_cnt_[0]++;
					local_map_->jointRef = global_goal_;
					local_solver_->getProblem()->setTau(1e-5);
					if (localSolve(start_motion, NULL, new_motion))
					{
						suc_cnt_[0]++;
						new_motion->parent = start_motion;
						nn_->add(new_motion);

						double dist = 0.0;
						bool sat = goal->isSatisfied(new_motion->state, &dist);
						if (sat)
						{
							solution = new_motion;
							break;
						}
						else
						{
							WARNING_NAMED("FRRT", "Goal check failed with error "<<dist);
						}
					}
				}
				///	If global not succeeded
				else
				{
					///	This start motion can not be connected to goal
					start_motion->setGlobalInvalid();
					///	Sample a random state
					bool r_ok = false;
					do
					{
						if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
						{
							goal_s->sampleGoal(new_motion->state);
						}
						else
						{
							sampler_->sampleUniform(new_motion->state);
						}
						r_ok = si_->getStateValidityChecker()->isValid(new_motion->state);
					}
					while (!r_ok && ptc == false);

					if (!r_ok)
					{
						OMPL_ERROR("Random sample failed");
						break;
					}
					Motion *near_motion = nn_->nearest(new_motion);
					bool valid = false;

					///	Do a regular line segment check
					ompl::base::State *last_valid_state = si_->allocState();
					si_->copyState(last_valid_state, near_motion->state);
					std::pair<ompl::base::State*, double> last_valid(last_valid_state, 0);
					try_cnt_[2]++;
					if (si_->checkMotion(near_motion->state, new_motion->state, last_valid))
					{
						suc_cnt_[2]++;
						new_motion->parent = near_motion;
						nn_->add(new_motion);
						valid = true;
					}
					///	Do a local try
					else
					{
						if (last_valid.second == 0)
							last_valid_state = NULL;
						try_cnt_[1]++;
						///	Set local solver goal
						Eigen::VectorXd eigen_g((int) si_->getStateDimension());
						memcpy(eigen_g.data(), new_motion->state->as<
								ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
								* eigen_g.rows());
						local_map_->jointRef = eigen_g;
						local_solver_->getProblem()->setTau(1e-2);
						if (localSolve(near_motion, last_valid_state, new_motion))
						{
							suc_cnt_[1]++;
							new_motion->parent = near_motion;
							nn_->add(new_motion);
							valid = true;
						}
					}

					if (valid)
					{
						newTry = true;
						start_motion = new_motion;
					}
				}
			}

			if (solution != NULL)
			{
				lastGoalMotion_ = solution;
				/* construct the solution path */
				std::vector<Motion*> mpath;
				while (solution != NULL)
				{
					if (solution->internal_path != nullptr)
					{
						for (int i = solution->internal_path->rows() - 1; i > 0; i--)
						{
							Motion *local_motion = new Motion(si_);
							Eigen::VectorXd tmp = solution->internal_path->row(i);
							memcpy(local_motion->state->as<
									ompl::base::RealVectorStateSpace::StateType>()->values, tmp.data(), sizeof(double)
									* (int) si_->getStateDimension());
							mpath.push_back(local_motion);
						}
						if (solution->inter_state != NULL)
						{
							Motion *local_motion = new Motion(si_);
							si_->copyState(local_motion->state, solution->inter_state);
							mpath.push_back(local_motion);
						}
					}
					else
					{
						mpath.push_back(solution);
					}
					solution = solution->parent;
				}
				PathGeometric *path = new PathGeometric(si_);
				for (int i = mpath.size() - 1; i >= 0; --i)
					path->append(mpath[i]->state);
				pdef_->addSolutionPath(base::PathPtr(path), false, 0, getName());
				OMPL_INFORM("Problem Solved");
				solved = true;
			}
			WARNING_NAMED("FRRT", "Created "<<nn_->size()<<" states");
			WARNING_NAMED("FRRT", "Global succeeded/try "<<suc_cnt_[0]<<"/"<<try_cnt_[0]);
			WARNING_NAMED("FRRT", "Local succeeded/try "<<suc_cnt_[1]<<"/"<<try_cnt_[1]);
			WARNING_NAMED("FRRT", "Normal succeeded/try "<<suc_cnt_[2]<<"/"<<try_cnt_[2]);
			return base::PlannerStatus(solved, false);
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
			if (!ok(ini.initialise(xml_file, ser, sol, prob, "LocalProblem", "FRRTLocal")))
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
			base::Goal *goal = pdef_->getGoal().get();
			if (!goal)
			{
				INDICATE_FAILURE
				return false;
			}
			if (prob->getTaskDefinitions().find("LocalTask") == prob->getTaskDefinitions().end())
			{
				ERROR("Missing XML tag of 'LocalTask'");
				return false;
			}
			if (prob->getTaskMaps().find("CSpaceMap") == prob->getTaskMaps().end())
			{
				ERROR("Missing XML tag of 'CSpaceMap'");
				return false;
			}
			local_task_ =
					boost::static_pointer_cast<exotica::TaskSqrError>(prob->getTaskDefinitions().at("LocalTask"));
			local_map_ =
					boost::static_pointer_cast<exotica::Identity>(prob->getTaskMaps().at("CSpaceMap"));
			collision_task_ =
					boost::static_pointer_cast<exotica::TaskSqrError>(prob->getTaskDefinitions().at("CollisionAvoidanceTask"));

			init_rho_[0] = local_task_->getRho(0);
			init_rho_[1] = collision_task_->getRho(0);
			return true;
		}

		bool FRRT::resetSceneAndGoal(const exotica::Scene_ptr & scene, const Eigen::VectorXd & goal)
		{
			global_goal_.setZero(goal.size());
			global_goal_ = goal;
			if (!ok(local_solver_->getProblem()->setScene(scene->getPlanningScene())))
			{
				INDICATE_FAILURE
				return false;
			}
			return true;
		}

		bool FRRT::localSolve(Motion *sm, base::State *is, Motion *gm)
		{
			int dim = (int) si_->getStateDimension();
			Eigen::VectorXd qs(dim), qg(dim);
			memcpy(qs.data(),
					is == NULL ? sm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values : is->as<
							ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
					* qs.rows());
			Eigen::MatrixXd local_path;
			if (ok(local_solver_->SolveFullSolution(qs, local_path)))
			{
				/* Local planner succeeded */
				gm->inter_state = is;
				gm->internal_path.reset(new Eigen::MatrixXd(local_path));
				gm->parent = sm;
				qg = local_path.row(local_path.rows() - 1).transpose();
				memcpy(gm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values, qg.data(), sizeof(double)
						* qg.rows());
				return true;
			}
			return false;
		}
	}	//	Namespace geometric
}	//	Namespace ompl

