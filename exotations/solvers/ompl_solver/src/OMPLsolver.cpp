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
#include "frrt/FRRT.h"

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

    std::string OMPLsolver::print(std::string prepend)
    {
        std::string ret = Object::print(prepend);
        ret+="\n"+prepend+"  Goal:";
        if(prob_)
          ret+="\n"+prob_->print(prepend+"    ");
        ret+="\n"+prepend+"  Cost:";
        if(costs_)
          ret+="\n"+costs_->print(prepend+"    ");
        ret+="\n"+prepend+"  Goal bias:";
        if(goalBias_)
          ret+="\n"+goalBias_->print(prepend+"    ");
        ret+="\n"+prepend+"  Sampling bias:";
        if(samplingBias_)
          ret+="\n"+samplingBias_->print(prepend+"    ");
        return ret;
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
		if (ok(state_space_->copyToOMPLState(ompl_start_state.get(), q0)))
		{
			ompl_simple_setup_->setStartState(ompl_start_state);

			// Solve here
			ompl::time::point start = ompl::time::now();
			preSolve();
            ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(timeout_	- ompl::time::seconds(ompl::time::now() - start));
			registerTerminationCondition(ptc);
			if (ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION)
			{
				double last_plan_time_ = ompl_simple_setup_->getLastPlanComputationTime();
				unregisterTerminationCondition();

				finishedSolving_ = true;

				postSolve();

				if (!ompl_simple_setup_->haveSolutionPath())
					return FAILURE;
				planning_time_ = ros::Time::now() - startTime;
				getSimplifiedPath(ompl_simple_setup_->getSolutionPath(), solution, timeout_
						- planning_time_.toSec());
				planning_time_ = ros::Time::now() - startTime;
				ompl::base::PlannerData data(ompl_simple_setup_->getSpaceInformation());
				ompl_simple_setup_->getPlanner()->getPlannerData(data);

				succ_cnt_++;
//				result_file_ << succ_cnt_ << " " << planning_time_.toSec() << " " << data.numVertices() << std::endl;
//				HIGHLIGHT_NAMED("OMPLBenchmarking", "Success No "<<succ_cnt_<<", Time: "<<planning_time_.toSec()<<", Nodes: "<<data.numVertices());
				return SUCCESS;
			}
			else
			{
				finishedSolving_ = true;
				postSolve();
				planning_time_ = ros::Time::now() - startTime;
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
        return ob::GoalPtr((OMPLGoal*)new OMPLGoalSampler(ompl_simple_setup_->getSpaceInformation(), prob_, goalBias_));
	}

	void OMPLsolver::startSampling()
    {
		bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
		if (gls)
		{
			static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->startSampling();
        }
	}

	void OMPLsolver::stopSampling()
	{
		bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
		if (gls)
        {
			static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->stopSampling();
        }

	}

	void OMPLsolver::preSolve()
	{
		// clear previously computed solutions
		ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
		const ob::PlannerPtr planner = ompl_simple_setup_->getPlanner();
        if (planner) planner->clear();
        startSampling();
		ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
	}

	void OMPLsolver::postSolve()
	{
        stopSampling();
        int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
        int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
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

	EReturn OMPLsolver::getSimplifiedPath(ompl::geometric::PathGeometric &pg,
			Eigen::MatrixXd & traj, double d)
	{
        if (smooth_->data)
		{
			int original_cnt = pg.getStateCount();
			ros::Time start = ros::Time::now();
			ompl_simple_setup_->simplifySolution(d);
			if (ompl_simple_setup_->haveSolutionPath())
			{
				pg.interpolate();
			}
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
					INFO("Using planning algorithm: "<<selected_planner_<<". Trajectory smoother is "<<(smooth_->data?"Enabled":"Disabled"));
					known = true;
					break;
				}
			}
		}
		XML_CHECK("max_goal_sampling_attempts");
		XML_OK(getInt(*xmltmp, goal_ampling_max_attempts_));

		projection_joints_.clear();
		tmp_handle = handle.FirstChildElement("ProjectionJoints");
		if (tmp_handle.ToElement())
		{
			tinyxml2::XMLHandle jnt_handle = tmp_handle.FirstChildElement("joint");
			while (jnt_handle.ToElement())
			{
				const char* atr = jnt_handle.ToElement()->Attribute("name");
				if (atr)
				{
					projection_joints_.push_back(std::string(atr));
				}
				else
				{
					INDICATE_FAILURE
					return FAILURE;
				}
				jnt_handle = jnt_handle.NextSiblingElement("joint");
			}
		}

		std::string path = ros::package::getPath("ompl_solver") + "/result/result.txt";
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

    EReturn OMPLsolver::specifyProblem(PlanningProblem_ptr goals, PlanningProblem_ptr costs, PlanningProblem_ptr goalBias, PlanningProblem_ptr samplingBias)
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
				INDICATE_FAILURE				;
				return FAILURE;
			}
		}

        if (costs)
        {
            for (auto & it : costs_->getScenes())
            {
                if (!ok(it.second->activateTaskMaps()))
                {
                    INDICATE_FAILURE				;
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
                    INDICATE_FAILURE				;
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
                    INDICATE_FAILURE				;
                    return FAILURE;
                }
            }
        }

		state_space_ = OMPLStateSpace::FromProblem(prob_);
		ompl_simple_setup_.reset(new og::SimpleSetup(state_space_));


		ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new OMPLStateValidityChecker(this)));
		ompl_simple_setup_->setPlannerAllocator(boost::bind(known_planners_[selected_planner_], _1, this->getObjectName()));
        ompl_simple_setup_->getSpaceInformation()->setup();

		std::vector<std::string> jnts;
		prob_->getScenes().begin()->second->getJointNames(jnts);
		if (projection_joints_.size() > 0)
		{
			bool projects_ok_ = true;
			std::vector<int> vars(projection_joints_.size());
			for (int i = 0; i < projection_joints_.size(); i++)
			{
				bool found = false;
				for (int j = 0; j < jnts.size(); j++)
				{
					if (projection_joints_[i].compare(jnts[j]) == 0)
					{
						vars[i] = j;
						found = true;
						break;
					}
				}
				if (!found)
				{
					WARNING("Projection joint ["<<projection_joints_[i]<<"] does not exist, OMPL Projection Evaluator not used");
					projects_ok_ = false;
					break;
				}
			}
			if (projects_ok_)
			{
				ompl_simple_setup_->getStateSpace()->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new exotica::OMPLProjection(state_space_, vars)));
				std::string tmp;
				for (int i = 0; i < projection_joints_.size(); i++)
					tmp = tmp + "[" + projection_joints_[i] + "] ";
				HIGHLIGHT_NAMED(object_name_, " Using projection joints "<<tmp);
			}
		}

        ompl_simple_setup_->setGoal(constructGoal());
        if (ompl_simple_setup_->getGoal())
            ompl_simple_setup_->setup();

		if (selected_planner_.compare("geometric::FRRT") == 0)
		{
			INFO_NAMED(object_name_, "Setting up FRRT Local planner from file\n"<<prob_->local_planner_config_);
			if (!ompl_simple_setup_->getPlanner()->as<ompl::geometric::FRRT>()->setUpLocalPlanner(prob_->local_planner_config_, prob_->scenes_.begin()->second))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}
		return SUCCESS;
	}

	EReturn OMPLsolver::resetIfNeeded()
	{
		if (selected_planner_.compare("geometric::FRRT") == 0)
		{
			if (!ompl_simple_setup_->getPlanner()->as<ompl::geometric::FRRT>()->resetSceneAndGoal(prob_->scenes_.begin()->second, boost::static_pointer_cast<
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
        registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2));
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

		registerPlannerAllocator("geometric::FRRT", boost::bind(&allocatePlanner<og::FRRT>, _1, _2));
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
