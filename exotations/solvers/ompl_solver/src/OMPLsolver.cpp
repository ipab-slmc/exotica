/*
 * OMPLsolver.cpp
 *
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
 */

#include "ompl_solver/OMPLsolver.h"

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
// #include <ompl/geometric/planners/fmt/FMT.h> // Broken?
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include "frrt/FRRT.h"
#include "frrt/BFRRT.h"
#include "frrt/FRRTConnect.h"
//#include "frrt/FKPIECE.h"

//#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/tools/config/MagicConstants.h>
#include "ompl_solver/OMPLGoalUnion.h"
#include "ompl_solver/OMPLGoal.h"
#include "ompl_solver/OMPLGoalSampler.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
#define MAT_OK(x) if(!ok(x)){INDICATE_FAILURE; exit(1);}

namespace exotica
{

	namespace ob = ompl::base;
	namespace og = ompl::geometric;
	namespace ot = ompl::tools;

	REGISTER_MOTIONSOLVER_TYPE("OMPLsolver", exotica::OMPLsolver);

	OMPLsolver::OMPLsolver() :
			timeout_(60.0), finishedSolving_(false)
	{
		registerDefaultPlanners();
	}

	OMPLsolver::~OMPLsolver()
	{
		// If this is not empty, your code is bad and you should feel bad!
		// Whoop whoop whoop whoop ...
	}

	std::string OMPLsolver::print(std::string prepend)
	{
		std::string ret = Object::print(prepend);
		ret += "\n" + prepend + "  Goal:";
		if (prob_)
			ret += "\n" + prob_->print(prepend + "    ");
		ret += "\n" + prepend + "  Cost:";
		if (costs_)
			ret += "\n" + costs_->print(prepend + "    ");
		ret += "\n" + prepend + "  Goal bias:";
		if (goalBias_)
			ret += "\n" + goalBias_->print(prepend + "    ");
		ret += "\n" + prepend + "  Sampling bias:";
		if (samplingBias_)
			ret += "\n" + samplingBias_->print(prepend + "    ");
		return ret;
	}

	void OMPLsolver::recordData()
	{
		ompl::base::PlannerData data(ompl_simple_setup_->getSpaceInformation());
		ompl_simple_setup_->getPlanner()->getPlannerData(data);
		int cnt = prob_->getScenes().begin()->second->getCollisionScene()->stateCheckCnt_;
		result_file_ << 1 << " " << planning_time_ << " " << data.numVertices() << " " << cnt;
		if (isFlexiblePlanner())
		{
			cnt =
					ompl_simple_setup_->getPlanner()->as<ompl::geometric::FlexiblePlanner>()->checkCnt_;
			result_file_ << " " << cnt;
		}
		result_file_ << std::endl;

		prob_->getScenes().begin()->second->getCollisionScene()->stateCheckCnt_ = 0;
	}
	void OMPLsolver::setMaxPlanningTime(double t)
	{
		timeout_ = t;
	}

	EReturn OMPLsolver::Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution)
	{
		ros::Time startTime = ros::Time::now();
		finishedSolving_ = false;

		ompl::base::ScopedState<> ompl_start_state(state_space_);
		if (ok(compound_ ? boost::static_pointer_cast<OMPLSE3RNCompoundStateSpace>(state_space_)->EigenToOMPLState(q0, ompl_start_state.get()) : boost::static_pointer_cast<
									OMPLStateSpace>(state_space_)->copyToOMPLState(ompl_start_state.get(), q0)))
		{

			ompl_simple_setup_->setStartState(ompl_start_state);
			preSolve();
			// Solve here
			ompl::time::point start = ompl::time::now();
			ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout_
					- ompl::time::seconds(ompl::time::now() - start));
			registerTerminationCondition(ptc);
			if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION)
			{
				double last_plan_time_ = ompl_simple_setup_->getLastPlanComputationTime();
				unregisterTerminationCondition();

				finishedSolving_ = true;

				if (!ompl_simple_setup_->haveSolutionPath())
					return FAILURE;
				planning_time_ = ros::Time::now() - startTime;
				getSimplifiedPath(ompl_simple_setup_->getSolutionPath(), solution, ptc);
				planning_time_ = ros::Time::now() - startTime;
				recordData();
				postSolve();
				succ_cnt_++;
				return SUCCESS;
			}
			else
			{
				finishedSolving_ = true;
				planning_time_ = ros::Time::now() - startTime;
				recordData();
				postSolve();
				return FAILURE;
			}
		}
		else
		{
			ERROR("Can't copy start state!");
			planning_time_ = ros::Time::now() - startTime;
			return FAILURE;
		}

	}
	bool OMPLsolver::getFinishedSolving()
	{
		return finishedSolving_;
	}

	int OMPLsolver::getGoalMaxAttempts()
	{
		return goal_ampling_max_attempts_;
	}

	ompl::base::GoalPtr OMPLsolver::constructGoal()
	{
		goal_state_.reset(new OMPLGoalState(ompl_simple_setup_->getSpaceInformation(), prob_));
		return ob::GoalPtr(goal_state_);
	}

	void OMPLsolver::startSampling()
	{
//		bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
//		if (gls)
//		{
//			static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->startSampling();
//		}
	}

	void OMPLsolver::stopSampling()
	{
//		bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
//		if (gls)
//		{
//			static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->stopSampling();
//		}

	}

	void OMPLsolver::preSolve()
	{
		// clear previously computed solutions
		ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
		const ob::PlannerPtr planner = ompl_simple_setup_->getPlanner();
		if (planner)
			planner->clear();
		startSampling();
		ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
		ompl_simple_setup_->getPlanner()->setProblemDefinition(ompl_simple_setup_->getProblemDefinition());
	}

	void OMPLsolver::postSolve()
	{
		ompl_simple_setup_->clearStartStates();
		goal_state_->clearGoalState();
		int v =
				ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
		int iv =
				ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
		logDebug("There were %d valid motions and %d invalid motions.", v, iv);

		if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
			logWarn("Computed solution is approximate");
	}

	EReturn OMPLsolver::convertPath(const ompl::geometric::PathGeometric &pg,
			Eigen::MatrixXd & traj)
	{
		traj.resize(pg.getStateCount(), state_space_->getDimension());
		Eigen::VectorXd tmp(state_space_->getDimension());

		for (int i = 0; i < (int) pg.getStateCount(); ++i)
		{
			if (!ok(compound_ ? boost::static_pointer_cast<OMPLSE3RNCompoundStateSpace>(state_space_)->OMPLStateToEigen(pg.getState(i), tmp) : boost::static_pointer_cast<
										OMPLStateSpace>(state_space_)->copyFromOMPLState(pg.getState(i), tmp)))
			{
				ERROR("Can't copy state "<<i);
				return FAILURE;
			}
			else
			{
				traj.row(i) = tmp;
			}
		}
		return SUCCESS;
	}

	EReturn OMPLsolver::getSimplifiedPath(ompl::geometric::PathGeometric &pg,
			Eigen::MatrixXd & traj, ob::PlannerTerminationCondition &ptc)
	{
		if (smooth_->data)
		{
			HIGHLIGHT("Simplifying solution");
			int original_cnt = pg.getStateCount();
			ros::Time start = ros::Time::now();

			//ompl_simple_setup_->simplifySolution(d);
			// Lets do our own simplifier ~:)
			if (original_cnt >= 3)
			{
				og::PathSimplifierPtr psf_ = ompl_simple_setup_->getPathSimplifier();
				const ob::SpaceInformationPtr &si = ompl_simple_setup_->getSpaceInformation();

				bool tryMore = false;
				if (ptc == false)
					tryMore = psf_->reduceVertices(pg);
				if (ptc == false)
					psf_->collapseCloseVertices(pg);
				int times = 0;
				while (tryMore && ptc == false)
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
					times = 0;
					while (tryMore && ptc == false)
					{
						tryMore = psf_->shortcutPath(pg);
						times++;
					}
				}

				std::vector<ob::State*> &states = pg.getStates();
				//	Calculate number of states required
				unsigned int length = 0;
				const int n1 = states.size() - 1;
				for (int i = 0; i < n1; ++i)
					length += si->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
//				//	Forward reducing
//				HIGHLIGHT("States before forward reducing: "<<pg.getStateCount());
//				pg.interpolate(length);
//
//				bool need_backward = true;
//				for (int i = states.size() - 1; i > 0; i--)
//				{
//					if (si->checkMotion(states[0], states[i]))
//					{
//						ob::State *start = si->cloneState(states[0]);
//						pg.keepAfter(states[i]);
//						pg.prepend(start);
//						need_backward = (i == states.size() - 1) ? false : true;
//						break;
//					}
//				}
//
//				//	Backward reducing
//				ob::State *mid;
//				if (need_backward)
//				{
//					HIGHLIGHT("States before backward reducing: "<<pg.getStateCount());
//					pg.interpolate(length);
//					for (int i = 1; i < states.size(); i++)
//					{
//						if (si->checkMotion(states[states.size() - 1], states[i]))
//						{
//							ob::State *goal = si->cloneState(states[states.size() - 1]);
//							pg.keepBefore(states[i]);
//							pg.append(goal);
//							mid = si->cloneState(states[i]);
//							break;
//						}
//					}
//				}

				pg.interpolate(length);
			}

//			if (ompl_simple_setup_->haveSolutionPath())
//			{
//				pg.interpolate();
//			}
			HIGHLIGHT_NAMED("OMPLSolver", "Simplification took "<<ros::Duration(ros::Time::now()-start).toSec()<<"sec. States: "<<original_cnt<<"->"<<pg.getStateCount());
		}
		convertPath(pg, traj);

		return SUCCESS;
	}

	void OMPLsolver::getOriginalSolution(Eigen::MatrixXd & orig)
	{
		orig.resize(original_solution_.rows(), original_solution_.cols());
		orig = original_solution_;
	}

	EReturn OMPLsolver::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("TrajectorySmooth");
		server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, smooth_);

		tinyxml2::XMLElement* xmltmp;
		std::string txt;
		XML_CHECK("algorithm");
		{
			txt = std::string(xmltmp->GetText());
			bool known = false;
			std::vector<std::string> nm = getPlannerNames();
			for (std::string s : nm)
			{
				if (txt.compare(s.substr(11)) == 0)
				{
					selected_planner_ = s;
					INFO("Using planning algorithm: "<<selected_planner_<<". Trajectory smoother is "<<(smooth_->data?"Enabled":"Disabled"));
					known = true;
					break;
				}
			}
			if (!known)
			{
				ERROR("Unknown planning algorithm ["<<txt<<"]");
				HIGHLIGHT_NAMED(object_name_,"Available algorithms:");
				for(std::string s:nm)
					HIGHLIGHT_NAMED(object_name_,s);
				return FAILURE;
			}
		}

		XML_CHECK("max_goal_sampling_attempts");
		XML_OK(getInt(*xmltmp, goal_ampling_max_attempts_));

		projection_components_.clear();
		tmp_handle = handle.FirstChildElement("Projection");
		if (tmp_handle.ToElement())
		{
			projector_ = tmp_handle.ToElement()->Attribute("type");
			tinyxml2::XMLHandle jnt_handle = tmp_handle.FirstChildElement("component");
			while (jnt_handle.ToElement())
			{
				const char* atr = jnt_handle.ToElement()->Attribute("name");
				if (atr)
				{
					projection_components_.push_back(std::string(atr));
				}
				else
				{
					INDICATE_FAILURE
					return FAILURE;
				}
				jnt_handle = jnt_handle.NextSiblingElement("component");
			}
		}

		tmp_handle = handle.FirstChildElement("Range");
		if (tmp_handle.ToElement())
		{
			server_->registerParam<std_msgs::String>(ns_, tmp_handle, range_);
		}

		std::string path = ros::package::getPath("ompl_solver") + "/result/" + txt + ".txt";
		result_file_.open(path);
		if (!result_file_.is_open())
		{
			ERROR("Error open "<<path);
			return FAILURE;
		}
		succ_cnt_ = 0;
		return SUCCESS;
	}

	EReturn OMPLsolver::specifyProblem(PlanningProblem_ptr pointer)
	{
		return specifyProblem(pointer, NULL, NULL, NULL);
	}

	ompl::base::StateSamplerPtr allocSE3RNStateSampler(const ompl::base::StateSpace *ss)
	{
		return ob::StateSamplerPtr(new OMPLSE3RNCompoundStateSampler(ss));
	}
	EReturn OMPLsolver::specifyProblem(PlanningProblem_ptr goals, PlanningProblem_ptr costs,
			PlanningProblem_ptr goalBias, PlanningProblem_ptr samplingBias)
	{
		if (goals->type().compare(std::string("exotica::OMPLProblem")) != 0)
		{
			ERROR("This solver can't use problem of type '" << goals->type() << "'!");
			return PAR_INV;
		}
		if (costs && costs->type().compare(std::string("exotica::OMPLProblem")) != 0)
		{
			ERROR("This solver can't use problem of type '" << costs->type() << "'!");
			return PAR_INV;
		}
		if (goalBias && goalBias->type().compare(std::string("exotica::OMPLProblem")) != 0)
		{
			ERROR("This solver can't use problem of type '" << goalBias->type() << "'!");
			return PAR_INV;
		}
		if (samplingBias && samplingBias->type().compare(std::string("exotica::OMPLProblem")) != 0)
		{
			ERROR("This solver can't use problem of type '" << samplingBias->type() << "'!");
			return PAR_INV;
		}
		MotionSolver::specifyProblem(goals);
		prob_ = boost::static_pointer_cast<OMPLProblem>(goals);
		costs_ = boost::static_pointer_cast<OMPLProblem>(costs);
		goalBias_ = boost::static_pointer_cast<OMPLProblem>(goalBias);
		samplingBias_ = boost::static_pointer_cast<OMPLProblem>(samplingBias);

		for (auto & it : prob_->getScenes())
		{
			if (!ok(it.second->activateTaskMaps()))
			{
				INDICATE_FAILURE
				;
				return FAILURE;
			}
		}

		if (costs)
		{
			for (auto & it : costs_->getScenes())
			{
				if (!ok(it.second->activateTaskMaps()))
				{
					INDICATE_FAILURE
					;
					return FAILURE;
				}
			}
		}

		if (goalBias_)
		{
			for (auto & it : goalBias_->getScenes())
			{
				if (!ok(it.second->activateTaskMaps()))
				{
					INDICATE_FAILURE
					;
					return FAILURE;
				}
			}
		}

		if (samplingBias_)
		{
			for (auto & it : samplingBias_->getScenes())
			{
				if (!ok(it.second->activateTaskMaps()))
				{
					INDICATE_FAILURE
					;
					return FAILURE;
				}
			}
		}

		compound_ = prob_->isCompoundStateSpace();
		if (compound_)
		{
			state_space_ =
					ob::StateSpacePtr(OMPLSE3RNCompoundStateSpace::FromProblem(prob_, server_));
		}
		else
		{
			state_space_ = OMPLStateSpace::FromProblem(prob_);
		}

		HIGHLIGHT_NAMED(object_name_,"Using state space: "<<state_space_->getName());
		ompl_simple_setup_.reset(new og::SimpleSetup(state_space_));

		ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new OMPLStateValidityChecker(this)));
		ompl_simple_setup_->setPlannerAllocator(boost::bind(known_planners_[selected_planner_], _1, this->getObjectName()
				+ "_" + selected_planner_));
		if (compound_)
			ompl_simple_setup_->getStateSpace()->setStateSamplerAllocator(boost::bind(&allocSE3RNStateSampler, (ob::StateSpace*) state_space_.get()));
		ompl_simple_setup_->getSpaceInformation()->setup();

		std::vector<std::string> jnts;
		prob_->getScenes().begin()->second->getJointNames(jnts);
		if (projector_.compare("Joints") == 0)
		{
			bool projects_ok_ = true;
			std::vector<int> vars(projection_components_.size());
			for (int i = 0; i < projection_components_.size(); i++)
			{
				bool found = false;
				for (int j = 0; j < jnts.size(); j++)
				{
					if (projection_components_[i].compare(jnts[j]) == 0)
					{
						vars[i] = j;
						found = true;
						break;
					}
				}
				if (!found)
				{
					WARNING("Projection joint ["<<projection_components_[i]<<"] does not exist, OMPL Projection Evaluator not used");
					projects_ok_ = false;
					break;
				}
			}
			if (projects_ok_)
			{
				ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new exotica::OMPLProjection(state_space_, vars)));
				std::string tmp;
				for (int i = 0; i < projection_components_.size(); i++)
					tmp = tmp + "[" + projection_components_[i] + "] ";
				HIGHLIGHT_NAMED(object_name_, " Using projection joints "<<tmp);
			}
		}
		else if (projector_.compare("DMesh") == 0)
		{
			//	Construct default DMesh projection relationship
			std::vector<std::pair<int, int> > tmp_index(projection_components_.size() - 2);
			for (int i = 0; i < tmp_index.size(); i++)
			{
				tmp_index[i].first = i;
				tmp_index[i].second = tmp_index.size();
			}
			ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new exotica::DMeshProjection(state_space_, projection_components_, tmp_index, prob_->getScenes().begin()->second)));
			std::string tmp;
			for (int i = 0; i < projection_components_.size(); i++)
				tmp = tmp + "[" + projection_components_[i] + "] ";
			HIGHLIGHT_NAMED(object_name_, " Using DMesh Projection links "<<tmp);
		}
		else
		{
			WARNING_NAMED(object_name_, "Unknown projection type "<<projector_<<". Setting ProjectionEvaluator failed.");
		}
		ompl_simple_setup_->setGoal(constructGoal());
		ompl_simple_setup_->setup();

		if (isFlexiblePlanner())
		{
			INFO_NAMED(object_name_, "Setting up FRRT Local planner from file\n"<<prob_->local_planner_config_);
			if (!ompl_simple_setup_->getPlanner()->as<ompl::geometric::FlexiblePlanner>()->setUpLocalPlanner(prob_->local_planner_config_, prob_->scenes_.begin()->second))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}
		if (selected_planner_.compare("geometric::pFRRT") == 0)
		{
			ompl_simple_setup_->getPlanner()->as<ompl::geometric::pRRT>()->setThreadCount(10);
		}
		if (range_ && ompl_simple_setup_->getPlanner()->params().hasParam("range"))
			ompl_simple_setup_->getPlanner()->params().setParam("range", range_->data);

		return SUCCESS;
	}

	EReturn OMPLsolver::resetIfNeeded()
	{
		if (isFlexiblePlanner())
		{
			if (!ompl_simple_setup_->getPlanner()->as<ompl::geometric::FlexiblePlanner>()->resetSceneAndGoal(prob_->scenes_.begin()->second, boost::static_pointer_cast<
					exotica::Identity>(prob_->getTaskMaps().at("CSpaceGoalMap"))->jointRef))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}
		return SUCCESS;
	}

	bool OMPLsolver::isSolvable(const PlanningProblem_ptr & prob)
	{
		if (prob->type().compare("exotica::OMPLProblem") == 0)
			return true;
		return false;
	}

	template<typename T> static ompl::base::PlannerPtr allocatePlanner(
			const ob::SpaceInformationPtr &si, const std::string &new_name)
	{
		ompl::base::PlannerPtr planner(new T(si));
		if (!new_name.empty())
			planner->setName(new_name);
		planner->setup();
		return planner;
	}

	void OMPLsolver::registerDefaultPlanners()
	{
		registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2));
		registerPlannerAllocator("geometric::pRRT", boost::bind(&allocatePlanner<og::pRRT>, _1, _2));
		registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<
				og::RRTConnect>, _1, _2));
		registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2));
		registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2));
		registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2));
		registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2));
		registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2));
		registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2));
		registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2));
		registerPlannerAllocator("geometric::RRTStar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2));
		registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2));
		registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2));
		//registerPlannerAllocator("geometric::FMT", boost::bind(&allocatePlanner<og::FMT>, _1, _2));
		registerPlannerAllocator("geometric::PDST", boost::bind(&allocatePlanner<og::PDST>, _1, _2));
		registerPlannerAllocator("geometric::LazyPRM", boost::bind(&allocatePlanner<og::LazyPRM>, _1, _2));
		registerPlannerAllocator("geometric::SPARS", boost::bind(&allocatePlanner<og::SPARS>, _1, _2));
		registerPlannerAllocator("geometric::SPARStwo", boost::bind(&allocatePlanner<og::SPARStwo>, _1, _2));
		registerPlannerAllocator("geometric::LBTRRT", boost::bind(&allocatePlanner<og::LBTRRT>, _1, _2));
		registerPlannerAllocator("geometric::STRIDE", boost::bind(&allocatePlanner<og::STRIDE>, _1, _2));

		registerPlannerAllocator("geometric::FRRT", boost::bind(&allocatePlanner<og::FRRT>, _1, _2));
		registerPlannerAllocator("geometric::BFRRT", boost::bind(&allocatePlanner<og::BFRRT>, _1, _2));
		registerPlannerAllocator("geometric::FRRTConnect", boost::bind(&allocatePlanner<
				og::FRRTConnect>, _1, _2));
//		registerPlannerAllocator("geometric::FKPIECE", boost::bind(&allocatePlanner<og::FKPIECE>, _1, _2));

	}

	bool OMPLsolver::isFlexiblePlanner()
	{
		if (selected_planner_.compare("geometric::FRRT") == 0
				|| selected_planner_.compare("geometric::BFRRT") == 0
				|| selected_planner_.compare("geometric::FRRTConnect") == 0
				|| selected_planner_.compare("geometric::FKPIECE") == 0)
			return true;
		return false;
	}

	std::vector<std::string> OMPLsolver::getPlannerNames()
	{
		std::vector<std::string> ret;
		for (auto name : known_planners_)
		{
			ret.push_back(name.first);
		}
		return ret;
	}

	void OMPLsolver::registerTerminationCondition(const ob::PlannerTerminationCondition &ptc)
	{
		boost::mutex::scoped_lock slock(ptc_lock_);
		ptc_ = &ptc;
	}

	void OMPLsolver::unregisterTerminationCondition()
	{
		boost::mutex::scoped_lock slock(ptc_lock_);
		ptc_ = NULL;
	}

	bool OMPLsolver::terminate()
	{
		boost::mutex::scoped_lock slock(ptc_lock_);
		if (ptc_)
			ptc_->terminate();
		return true;
	}

	EReturn OMPLsolver::setGoalState(const Eigen::VectorXd & qT, const double eps)
	{
		LOCK(goal_lock_);
		ompl::base::ScopedState<> gs(state_space_);
		if (ok(compound_ ? boost::static_pointer_cast<OMPLSE3RNCompoundStateSpace>(state_space_)->EigenToOMPLState(qT, gs.get()) : boost::static_pointer_cast<
									OMPLStateSpace>(state_space_)->copyToOMPLState(gs.get(), qT)))
		{
			if (!goal_state_->isSatisfied(gs.get()))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			goal_state_->setState(gs);
			goal_state_->setThreshold(eps);
		}
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		return SUCCESS;
	}
} /* namespace exotica */
