/*
 * FRRTConnect.cpp
 *
 *  Created on: 5 Jun 2015
 *      Author: yiming
 */

#include "frrt/FRRTConnect.h"

namespace ompl
{
	namespace geometric
	{
		FRRTConnect::FRRTConnect(const base::SpaceInformationPtr & si) :
				base::Planner(si, "FRRTConnect")
		{
			specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
			specs_.directed = true;
			maxDistance_ = 0.0;
			Planner::declareParam<double>("range", this, &FRRTConnect::setRange, &FRRTConnect::getRange, "0.:1.:10000.");
			connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);
		}

		FRRTConnect::~FRRTConnect()
		{
			freeMemory();
		}

		void FRRTConnect::setup()
		{
			Planner::setup();
			ompl::tools::SelfConfig sc(si_, getName());
			sc.configurePlannerRange(maxDistance_);
			if (!tStart_)
				tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
			if (!tGoal_)
				tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
			tStart_->setDistanceFunction(boost::bind(&FRRTConnect::distanceFunction, this, _1, _2));
			tGoal_->setDistanceFunction(boost::bind(&FRRTConnect::distanceFunction, this, _1, _2));
		}

		void FRRTConnect::freeMemory()
		{
			std::vector<Motion*> motions;
			if (tStart_)
			{
				tStart_->list(motions);
				for (unsigned int i = 0; i < motions.size(); ++i)
				{
					if (motions[i]->state)
						si_->freeState(motions[i]->state);
					if (motions[i]->inter_state)
						si_->freeState(motions[i]->inter_state);
					delete motions[i];
				}
			}
			if (tGoal_)
			{
				tGoal_->list(motions);
				for (unsigned int i = 0; i < motions.size(); ++i)
				{
					if (motions[i]->state)
						si_->freeState(motions[i]->state);
					if (motions[i]->inter_state)
						si_->freeState(motions[i]->inter_state);
					delete motions[i];
				}
			}
		}

		void FRRTConnect::clear()
		{
			Planner::clear();
			sampler_.reset();
			freeMemory();
			if (tStart_)
				tStart_->clear();
			if (tGoal_)
				tGoal_->clear();
			connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);
		}

		FRRTConnect::GrowState FRRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
				Motion *rmotion)
		{
			/* find closest state in the tree */
			Motion *nmotion = tree->nearest(rmotion);

			/* assume we can reach the state we go towards */
			bool reach = true;

			/* find state to add */
			base::State *dstate = rmotion->state;
			double d = si_->distance(nmotion->state, rmotion->state);
			if (d > maxDistance_)
			{
				si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
				dstate = tgi.xstate;
				reach = false;
			}

			Motion *last_valid_motion = new Motion(si_);
			std::pair<ompl::base::State*, double> last_valid(last_valid_motion->state, 0);

			// if we are in the start tree, we just check the motion like we normally do;
			// if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
			// so we check that one first
			bool validMotion =
					tgi.start ? si_->checkMotion(nmotion->state, dstate, last_valid) : si_->getStateValidityChecker()->isValid(dstate)
										&& si_->checkMotion(dstate, nmotion->state, last_valid);

			//	Give local solver a try
			Motion *motion = new Motion(si_);
			if (!validMotion)
			{
				Eigen::VectorXd q_goal(si_->getStateDimension());
				memcpy(q_goal.data(),
						tgi.start ? dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values : nmotion->state->as<
								ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
						* q_goal.rows());

				local_map_->jointRef = q_goal;
				local_solver_->getProblem()->setTau(1e-4);
				Motion * s_motion = new Motion(si_);
				si_->copyState(s_motion->state, (tgi.start ? nmotion->state : dstate));
				if (localSolve(s_motion, last_valid_motion->state, motion))
				{
					validMotion = true;
				}
				else if (motion->internal_path)
				{
					validMotion = true;
				}
				si_->freeState(s_motion->state);
				delete s_motion;
			}
			if (validMotion)
			{
				/* create a motion */
				si_->copyState(motion->state, dstate);
				motion->parent = nmotion;
				motion->root = nmotion->root;
				tgi.xmotion = motion;

				tree->add(motion);
				if (reach)
					return REACHED;
				else
					return ADVANCED;
			}
			else
				return TRAPPED;
		}

		base::PlannerStatus FRRTConnect::solve(const base::PlannerTerminationCondition &ptc)
		{
			checkValidity();
			base::GoalSampleableRegion *goal =
					dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

			if (!goal)
			{
				OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
				return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
			}

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

			if (!sampler_)
				sampler_ = si_->allocStateSampler();

			OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int )(tStart_->size()
					+ tGoal_->size()));

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
					const base::State *st =
							tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
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

				GrowState gs = growTree(tree, tgi, rmotion);

				if (gs != TRAPPED)
				{
					/* remember which motion was just added */
					Motion *addedMotion = tgi.xmotion;

					/* attempt to connect trees */

					/* if reached, it means we used rstate directly, no need top copy again */
					if (gs != REACHED)
						si_->copyState(rstate, tgi.xstate);

					GrowState gsc = ADVANCED;
					tgi.start = startTree;
					while (gsc == ADVANCED)
						gsc = growTree(otherTree, tgi, rmotion);

					Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
					Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

					/* if we connected the trees in a valid way (start and goal pair is valid)*/
					if (gsc == REACHED
							&& goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
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
						std::vector<Motion*> mpath1;
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
									mpath1.push_back(local_motion);
								}
								if (solution->inter_state != NULL)
								{
									Motion *local_motion = new Motion(si_);
									si_->copyState(local_motion->state, solution->inter_state);
									mpath1.push_back(local_motion);
								}
							}
							else
								mpath1.push_back(solution);
							solution = solution->parent;
						}

						solution = goalMotion;
						std::vector<Motion*> mpath2;
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
									mpath2.push_back(local_motion);
								}
								if (solution->inter_state != NULL)
								{
									Motion *local_motion = new Motion(si_);
									si_->copyState(local_motion->state, solution->inter_state);
									mpath2.push_back(local_motion);
								}
							}
							else
								mpath2.push_back(solution);
							solution = solution->parent;
						}

						PathGeometric *path = new PathGeometric(si_);
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

			OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size()
					+ tGoal_->size(), tStart_->size(), tGoal_->size());

			return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
		}

		bool FRRTConnect::setUpLocalPlanner(const std::string & xml_file,
				const exotica::Scene_ptr & scene)
		{
			exotica::Initialiser ini;
			exotica::PlanningProblem_ptr prob;
			exotica::MotionSolver_ptr sol;
			if (!ok(ini.initialise(xml_file, ser_, sol, prob, "LocalProblem", "FRRTLocal")))
			{
				INDICATE_FAILURE
				return false;
			}
			if (sol->type().compare("exotica::IKsolver") == 0)
			{
				HIGHLIGHT_NAMED("FRRT", "Using local planner "<<sol->object_name_<<" at "<<sol.get());
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
			return true;
		}

		bool FRRTConnect::resetSceneAndGoal(const exotica::Scene_ptr & scene,
				const Eigen::VectorXd & goal)
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

		void FRRTConnect::getPlannerData(base::PlannerData &data) const
		{
			Planner::getPlannerData(data);

			std::vector<Motion*> motions;
			if (tStart_)
				tStart_->list(motions);

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
			if (tGoal_)
				tGoal_->list(motions);

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

		bool FRRTConnect::localSolve(Motion *sm, base::State *is, Motion *gm)
		{
			int dim = (int) si_->getStateDimension();
			Eigen::VectorXd qs(dim), qg(dim);
			memcpy(qs.data(),
					is == NULL ? sm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values : is->as<
							ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
					* qs.rows());
			Eigen::MatrixXd local_path;
			exotica::EReturn ret = local_solver_->SolveFullSolution(qs, local_path);
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
	}
}

