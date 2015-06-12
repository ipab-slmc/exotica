/*
 * BFRRT.cpp
 *
 *  Created on: 1 Jun 2015
 *      Author: yiming
 */

#include "frrt/BFRRT.h"

namespace ompl
{
	namespace geometric
	{
		BFRRT::BFRRT(const base::SpaceInformationPtr & si) :
				FlexiblePlanner(si, "BFRRT")
		{
			maxDistance_ = 0.0;
			Planner::declareParam<double>("range", this, &BFRRT::setRange, &BFRRT::getRange, "0.:1.:10000.");
			connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);
		}

		BFRRT::~BFRRT()
		{
			freeMemory();
		}

		void BFRRT::setup()
		{
			Planner::setup();
			ompl::tools::SelfConfig sc(si_, getName());
			sc.configurePlannerRange(maxDistance_);
			if (!tStart_)
				tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
			if (!tGoal_)
				tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
			tStart_->setDistanceFunction(boost::bind(&BFRRT::distanceFunction, this, _1, _2));
			tGoal_->setDistanceFunction(boost::bind(&BFRRT::distanceFunction, this, _1, _2));
		}

		void BFRRT::freeMemory()
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

		void BFRRT::clear()
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

		BFRRT::GrowState BFRRT::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
		{
			/* find closest state in the tree */
			Motion *nmotion = tree->nearest(rmotion);

			/* assume we can reach the state we go towards */
			bool reach = true;

			/* find state to add */
			base::State *dstate = rmotion->state;
			double d = si_->distance(nmotion->state, rmotion->state);
			// if we are in the start tree, we just check the motion like we normally do;
			// if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
			// so we check that one first
			bool validMotion =
					tgi.start ? si_->checkMotion(nmotion->state, dstate) : si_->getStateValidityChecker()->isValid(dstate)
										&& si_->checkMotion(dstate, nmotion->state);

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
					return REACHED;
				else
					return ADVANCED;
			}
			else
				return TRAPPED;
		}

		base::PlannerStatus BFRRT::solve(const base::PlannerTerminationCondition &ptc)
		{
			checkValidity();
			base::GoalSampleableRegion *goal =
					dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

			if (!goal)
			{
				OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
				return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
			}
			TreeGrowingInfo tgi;
			while (const base::State *st = pis_.nextStart())
			{
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				motion->root = motion->state;
				tStart_->add(motion);
				tgi.last_s = motion;
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

			Motion *rmotion = new Motion(si_);
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
						tgi.last_g = motion;
					}

					if (tGoal_->size() == 0)
					{
						OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
						break;
					}
				}
				static double nearest_r = 0.05 * distanceFunction(tgi.last_s, tgi.last_g);
				///	Get a random state
				bool r_ok = false;
				do
				{
					sampler_->sampleUniform(rmotion->state);
					r_ok = si_->getStateValidityChecker()->isValid(rmotion->state);
				}
				while (!r_ok && ptc == false);
				Motion *nearest_s = tStart_->nearest(rmotion);
				Motion *nearest_g = tGoal_->nearest(rmotion);
				Motion *last_valid_motion = new Motion(si_);
				std::pair<ompl::base::State*, double> last_valid(last_valid_motion->state, 0);

				Motion *new_s = NULL;
				Motion *new_g = NULL;
				///	Connect random node to start tree
				bool succ_s = si_->checkMotion(nearest_s->state, rmotion->state, last_valid);
				if (succ_s)
				{
					Motion *motion = new Motion(si_);
					si_->copyState(motion->state, rmotion->state);
					motion->parent = nearest_s;
					tStart_->add(motion);
					new_s = motion;
				}
				else
				{
					if (last_valid.second == 0)
						last_valid_motion->state = NULL;
					Eigen::VectorXd eigen_g((int) si_->getStateDimension());
					memcpy(eigen_g.data(), rmotion->state->as<
							ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
							* eigen_g.rows());
					local_map_->jointRef = eigen_g;
					local_solver_->getProblem()->setTau(1e-4);
					Motion *new_motion = new Motion(si_);
					if (localSolve(nearest_s, last_valid_motion->state, new_motion))
					{
						new_s = new_motion;
						tStart_->add(new_motion);
						succ_s = true;
					}
					else if (new_motion->internal_path)
					{
						si_->copyState(rmotion->state, new_motion->state);
						bool addNew = true;
//						std::vector<Motion*> n_motions;
//						tStart_->nearestR(new_motion, nearest_r, n_motions);
//						for (int i = 0; i < n_motions.size(); i++)
//							if (!n_motions[i]->global_valid_)
//							{
//								addNew = false;
//								break;
//							}
						if (addNew)
						{
							new_motion->global_valid_ = false;
							tStart_->add(new_motion);
							si_->copyState(rmotion->state, new_motion->state);
						}
					}
				}

				///	For goal tree, do the same thing
				last_valid.second = 0;
				bool succ_g = si_->checkMotion(nearest_g->state, rmotion->state, last_valid);
				if (succ_g)
				{
					Motion *motion = new Motion(si_);
					si_->copyState(motion->state, rmotion->state);
					motion->parent = nearest_g;
					tGoal_->add(motion);
					new_g = motion;
				}
				else
				{
					if (last_valid.second == 0)
						last_valid_motion->state = NULL;
					Eigen::VectorXd eigen_g((int) si_->getStateDimension());
					memcpy(eigen_g.data(), rmotion->state->as<
							ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
							* eigen_g.rows());
					local_map_->jointRef = eigen_g;
					local_solver_->getProblem()->setTau(1e-4);
					Motion *new_motion = new Motion(si_);
					if (localSolve(nearest_g, last_valid_motion->state, new_motion))
					{
						new_g = new_motion;
						succ_g = true;
					}
					else if (new_motion->internal_path)
					{
						si_->copyState(rmotion->state, new_motion->state);
						bool addNew = true;
//						std::vector<Motion*> n_motions;
//						tGoal_->nearestR(new_motion, nearest_r, n_motions);
//						for (int i = 0; i < n_motions.size(); i++)
//							if (!n_motions[i]->global_valid_)
//							{
//								addNew = false;
//								break;
//							}
						if (addNew)
						{
							new_motion->global_valid_ = false;
							tGoal_->add(new_motion);
						}
					}
				}

				///	If succeeded both ways, the a solution is found
				if (succ_s && succ_g)
				{
					connectionPoint_ = std::make_pair(new_s->state, new_g->state);
					Motion *solution = new_s;
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

					solution = new_g;
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

			si_->freeState(rmotion->state);
			delete rmotion;

			OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size()
					+ tGoal_->size(), tStart_->size(), tGoal_->size());

			return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
		}

		void BFRRT::getPlannerData(base::PlannerData &data) const
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

		bool BFRRT::localSolve(Motion *sm, base::State *is, Motion *gm)
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
	}
}
