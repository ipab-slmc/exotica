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
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <ompl/base/goals/GoalLazySamples.h>
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

	void OMPLsolver::setMaxPlanningTime(double t)
	{
		timeout_ = t;
	}

	EReturn OMPLsolver::Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution)
	{
		finishedSolving_ = false;
		ompl::base::ScopedState<> ompl_start_state(state_space_);
		if (ok(state_space_->copyToOMPLState(ompl_start_state.get(), q0)))
		{
			ompl_simple_setup_->setStartState(ompl_start_state);
			ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new OMPLStateValidityChecker(this)));
			ompl_simple_setup_->setPlannerAllocator(boost::bind(known_planners_[selected_planner_], _1, this->getObjectName()));
			// call the setParams() after setup(), so we know what the params are
			ompl_simple_setup_->getSpaceInformation()->setup();
			//ompl_simple_setup_.getSpaceInformation()->params().setParams(cfg, true);
			// call setup() again for possibly new param values
			//ompl_simple_setup_.getSpaceInformation()->setup();

			if (ompl_simple_setup_->getGoal())
				ompl_simple_setup_->setup();

			// Solve here
			ompl::time::point start = ompl::time::now();
			preSolve();
			ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout_
					- ompl::time::seconds(ompl::time::now() - start));
			registerTerminationCondition(ptc);
			if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION)
			{
				double last_plan_time_ = ompl_simple_setup_->getLastPlanComputationTime();
				unregisterTerminationCondition();

				finishedSolving_ = true;

				postSolve();

				if (!ompl_simple_setup_->haveSolutionPath())
					return FAILURE;

				getSimplifiedPath(ompl_simple_setup_->getSolutionPath(), solution);
				return SUCCESS;
			}
			else
			{
				finishedSolving_ = true;
				postSolve();
				return FAILURE;
			}
		}
		else
		{
			ERROR("Can't copy start state!");
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
		return ob::GoalPtr(new OMPLGoal(ompl_simple_setup_->getSpaceInformation(), prob_));

		std::vector<ob::GoalPtr> goals;

		//ompl::base::SpaceInformationPtr spi(new ompl::base::SpaceInformation(boost::static_pointer_cast<ompl::base::StateSpace>(state_space_)));
		ob::GoalPtr g = ob::GoalPtr(new OMPLGoal(ompl_simple_setup_->getSpaceInformation(), prob_));
		//ob::GoalPtr g = ob::GoalPtr(new OMPLGoalSampler(ompl_simple_setup_->getSpaceInformation(), prob_, shared_from_this()));
		goals.push_back(g);

		if (!goals.empty())
			return ompl::base::GoalPtr(new OMPLGoalUnion(goals));
		//return goals.size() == 1 ? goals[0] : ompl::base::GoalPtr(new OMPLGoalUnion(goals));
		else
			logError("Unable to construct goal representation");

		return ob::GoalPtr();
	}

	void OMPLsolver::startSampling()
	{
		CHECK_EXECUTION
		;
		bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
		if (gls)
		{
			CHECK_EXECUTION
			;
			static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->startSampling();
			CHECK_EXECUTION
			;
		}
		else
		{
			// we know this is a GoalSampleableMux by elimination
			CHECK_EXECUTION
			;
			static_cast<OMPLGoalUnion*>(ompl_simple_setup_->getGoal().get())->startSampling();
			CHECK_EXECUTION
			;
		}
	}

	void OMPLsolver::stopSampling()
	{
		bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
		if (gls)
			static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->stopSampling();
		else
			// we know this is a GoalSampleableMux by elimination
			static_cast<OMPLGoalUnion*>(ompl_simple_setup_->getGoal().get())->stopSampling();
	}

	void OMPLsolver::preSolve()
	{
		// clear previously computed solutions
		ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
		const ob::PlannerPtr planner = ompl_simple_setup_->getPlanner();
		if (planner)
			planner->clear();
		//startSampling();
		ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
	}

	void OMPLsolver::postSolve()
	{
		//stopSampling();
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
			if (!ok(state_space_->copyFromOMPLState(pg.getState(i), tmp)))
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

	EReturn OMPLsolver::getSimplifiedPath(ompl::geometric::PathGeometric &pg, Eigen::MatrixXd & traj)
	{
		ROS_ERROR_STREAM("States before simplification: "<<pg.getStateCount()<<", length:"<<pg.length());
		ompl::geometric::PathSimplifier ps(ompl_simple_setup_->getSpaceInformation());

		ps.shortcutPath(pg);
		ps.smoothBSpline(pg);
		ROS_ERROR_STREAM("States after simplification: "<<pg.getStateCount()<<", length:"<<pg.length());
		convertPath(pg,traj);

		return SUCCESS;
	}

	EReturn OMPLsolver::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLElement* xmltmp;
		XML_CHECK("algorithm");
		{
			std::string txt = std::string(xmltmp->GetText());
			bool known = false;
			std::vector<std::string> nm = getPlannerNames();
			for (std::string s : nm)
			{
				if (txt.compare(s.substr(11)) == 0)
				{
					selected_planner_ = s;
					known = true;
					break;
				}
			}
		}
		XML_CHECK("max_goal_sampling_attempts");
		XML_OK(getInt(*xmltmp, goal_ampling_max_attempts_));
		return SUCCESS;
	}

	EReturn OMPLsolver::specifyProblem(PlanningProblem_ptr pointer)
	{
		if (pointer->type().compare(std::string("exotica::OMPLProblem")) != 0)
		{
			ERROR("This solver can't use problem of type '" << pointer->type() << "'!");
			return PAR_INV;
		}
		problem_ = pointer;
		prob_ = boost::static_pointer_cast<OMPLProblem>(pointer);
		state_space_ = OMPLStateSpace::FromProblem(prob_);
		ompl_simple_setup_.reset(new og::SimpleSetup(state_space_));
		ob::GoalPtr goal = constructGoal();
		if (goal)
		{
			ompl_simple_setup_->setGoal(goal);
			INFO("Goal has been set");
			return SUCCESS;
		}
		INDICATE_FAILURE
		return FAILURE;
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
		registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<
				og::RRTConnect>, _1, _2));
		registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2));
		registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2));
		registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2));
		registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2));
		registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2));
		registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2));
		registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2));
		registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2));
		registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2));
		registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2));
		//registerPlannerAllocator("geometric::FMT", boost::bind(&allocatePlanner<og::FMT>, _1, _2));
		registerPlannerAllocator("geometric::PDST", boost::bind(&allocatePlanner<og::PDST>, _1, _2));
		registerPlannerAllocator("geometric::LazyPRM", boost::bind(&allocatePlanner<og::LazyPRM>, _1, _2));
		registerPlannerAllocator("geometric::SPARS", boost::bind(&allocatePlanner<og::SPARS>, _1, _2));
		registerPlannerAllocator("geometric::SPARStwo", boost::bind(&allocatePlanner<og::SPARStwo>, _1, _2));
		registerPlannerAllocator("geometric::LBTRRT", boost::bind(&allocatePlanner<og::LBTRRT>, _1, _2));
		registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2));
		registerPlannerAllocator("geometric::STRIDE", boost::bind(&allocatePlanner<og::STRIDE>, _1, _2));
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

} /* namespace exotica */
