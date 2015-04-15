/*
 * EXOTicaPlannerManager.cpp
 *
 *  Created on: 17 May 2014
 *      Author: s0972326
 */

#include "exotica_moveit/EXOTicaPlannerManager.h"

namespace exotica
{

	EXOTicaPlannerManager::EXOTicaPlannerManager() :
			planning_interface::PlannerManager(), nh_("~")
	{
	}

	EXOTicaPlannerManager::~EXOTicaPlannerManager()
	{
		// TODO Auto-generated destructor stub
	}

	bool EXOTicaPlannerManager::initialize(const robot_model::RobotModelConstPtr& model,
			const std::string &ns)
	{
		Initialiser ini;

		std::string filename;
		nh_.getParam("/EXOTica/exotica_config", filename);
		ROS_ERROR_STREAM("Exotica config file: "<<filename);
		if (filename.length() == 0)
		{
			ROS_ERROR_STREAM("Can't load exotica config file!!");
			return false;
		}

		ini.listSolversAndProblems(filename, problems_, solvers_);
		return true;
	}

	bool EXOTicaPlannerManager::canServiceRequest(
			const planning_interface::MotionPlanRequest& req) const
	{
		// TODO: this is a dummy implementation
		//      capabilities.dummy = false;
		return true;
	}

	planning_interface::PlanningContextPtr EXOTicaPlannerManager::getPlanningContext(
			const planning_scene::PlanningSceneConstPtr& planning_scene,
			const planning_interface::MotionPlanRequest &req,
			moveit_msgs::MoveItErrorCodes &error_code) const
	{
		planning_interface::PlanningContextPtr context_;
		if (req.group_name.empty())
		{
			logError("No group specified to plan for");
			error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
			context_.reset();
			return context_;
		}

		error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

		if (!planning_scene)
		{
			logError("No planning scene supplied as input");
			context_.reset();
			return context_;
		}

		{
			std::string problem_name;
			std::string solver_name;

			bool found = false;
			for (std::string s : solvers_)
			{
				for (std::string p : problems_)
				{
					if (req.planner_id.compare(s + " - " + p) == 0)
					{
						found = true;
						problem_name = p;
						solver_name = s;
						break;
					}
				}
			}
			if (!found)
			{
				logError("Problem or solver not found!");
				context_.reset();
				return context_;
			}

			const robot_model::JointModelGroup* model_group =
					planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
			logDebug("Creating new planning context");
			context_.reset(new EXOTicaPlanningContext("EXOTICA", model_group->getName(), planning_scene->getRobotModel(), problem_name, solver_name));
		}
		if (context_)
		{
			context_->clear();
			robot_state::RobotStatePtr start_state =
					planning_scene->getCurrentStateUpdated(req.start_state);

			// Setup the context
			context_->setPlanningScene(planning_scene);
			context_->setMotionPlanRequest(req);
			boost::static_pointer_cast<EXOTicaPlanningContext>(context_)->setCompleteInitialState(*start_state);

			if (boost::static_pointer_cast<EXOTicaPlanningContext>(context_)->configure(planning_scene))
			{
				logDebug("%s: New planning context is set.", context_->getName().c_str());
				error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
			}
			else
			{
				logError("EXOTica encountered an error!");
				context_.reset();
			}
		}

		return context_;
	}

	void EXOTicaPlanningContext::setCompleteInitialState(
			const robot_state::RobotState &complete_initial_robot_state)
	{
		start_state_ = complete_initial_robot_state;
	}

	std::string EXOTicaPlannerManager::getDescription() const
	{
		return "EXOTica";
	}

	void EXOTicaPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
	{
		algs.clear();
		for (std::string s : solvers_)
		{
			for (std::string p : problems_)
			{
				algs.push_back(s + " - " + p);
			}
		}
	}

	EXOTicaPlanningContext::EXOTicaPlanningContext(const std::string &name,
			const std::string &group, const robot_model::RobotModelConstPtr& model,
			const std::string &problem_name, const std::string &solver_name) :
					planning_interface::PlanningContext(name, group),
					start_state_(model),
					tau_(0.0),
					nh_("~"),
					problem_name_(problem_name),
					solver_name_(solver_name)
	{
	}

	bool EXOTicaPlanningContext::configure(const planning_scene::PlanningSceneConstPtr & scene)
	{
		Initialiser ini;

		std::string filename;
		nh_.getParam("/EXOTica/exotica_config", filename);
		if (filename.length() == 0)
		{
			ROS_ERROR_STREAM("Can't load exotica config file!");
			return false;
		}
		config_file_ = filename;
//		ROS_INFO_STREAM("Loading exotica from: "<<filename);
//		exotica::AICOsolver_ptr tmp_sol;
//		PlanningProblem_ptr prob;
//		if (ok(ini.initialise(filename, ser_, sol, prob, problem_name_, solver_name_)))
//		{
//			if (sol->type().compare("exotica::AICOsolver") == 0)
//			{
//				tau_ = boost::static_pointer_cast<AICOProblem>(prob)->getTau();
//			}
//			else if (sol->type().compare("exotica::IKsolver") == 0)
//			{
//				tau_ = boost::static_pointer_cast<IKProblem>(prob)->getTau();
//			}
//			if (!ok(sol->specifyProblem(prob)))
//			{
//				INDICATE_FAILURE
//				;
//				return false;
//			}
//			if (!ok(prob->setScene(scene)))
//			{
//				INDICATE_FAILURE
//				;
//				return false;
//			}
//		}
//		else
//		{
//			ROS_WARN_STREAM("Could not initialize EXOTica!");
//			return false;
//		}

		client_ = nh_.serviceClient<exotica_moveit::ExoticaPlanning>("/exotica_planning");
		return true;
	}

	/** \brief Solve the motion planning problem and store the result in \e res. This function should not clear data structures before computing. The constructor and clear() do that. */
	bool EXOTicaPlanningContext::solve(planning_interface::MotionPlanResponse &res)
	{
		ros::WallTime start_time = ros::WallTime::now();

		const moveit::core::JointModelGroup* model_group =
				planning_scene_->getRobotModel()->getJointModelGroup(request_.group_name);
		ROS_ERROR_STREAM("Move group: '"<< model_group->getName() <<"'");
		std::vector<std::string> names = model_group->getJointModelNames();

		Eigen::VectorXd q0 = Eigen::VectorXd::Zero(names.size());
		std::string tmp_name = "";
		for (int i = 0; i < names.size(); i++)
		{
			q0(i) = *start_state_.getJointPositions(names[i]);
			tmp_name = tmp_name + " " + names[i];
		}
		ROS_ERROR_STREAM("Joints="<<tmp_name);
		ROS_ERROR_STREAM("q0="<<q0.transpose());

		Eigen::MatrixXd solution;
		bool found_solution = false;

//		if (sol->type().compare("exotica::AICOsolver") == 0)
//		{
//			found_solution = ok(boost::static_pointer_cast<AICOsolver>(sol)->Solve(q0, solution));
//		}
//		else if (sol->type().compare("exotica::IKsolver") == 0)
//		{
//			found_solution = ok(boost::static_pointer_cast<IKsolver>(sol)->Solve(q0, solution));
//		}
//
//		if (found_solution)
//		{
//			ROS_WARN_STREAM("Solution found.");
//			res.trajectory_.reset(new robot_trajectory::RobotTrajectory(planning_scene_->getRobotModel(), model_group->getName()));
//			copySolution(solution, res.trajectory_.get());
//			res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
//			res.planning_time_ = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
//			solution_ = solution;
//			return true;
//		}
//		else
//		{
//			res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
//			return false;
//		}

		exotica_moveit::ExoticaPlanning srv;
		vectorEigenToExotica(q0, srv.request.q0);
		srv.request.xml_file_ = config_file_;
		moveit_msgs::PlanningScene tmp;
		planning_scene_->getPlanningSceneMsg(tmp);
		srv.request.scene_ = tmp;
		srv.request.group_name_ =request_.group_name;
		srv.request.max_time_ = getMotionPlanRequest().allowed_planning_time;
		if (!client_.waitForExistence(ros::Duration(5)))
		{
			ROS_ERROR("Exotica Planning service does not exist");
		}
		else
			ROS_ERROR("Calling Exotica Planning service");
		if (!client_.isValid())
			ROS_ERROR("Exotica Planning Client not valid");

		if (client_.call(srv))
		{
			ROS_ERROR("Calling Exotica Planning service succeeded");
			if (srv.response.succeeded_)
			{
				if (ok(matrixExoticaToEigen(srv.response.solution_, solution)))
				{
					res.trajectory_.reset(new robot_trajectory::RobotTrajectory(planning_scene_->getRobotModel(), model_group->getName()));
					res.planning_time_ = srv.response.planning_time_;
					copySolution(solution, res.trajectory_.get());
					solution_ = solution;
					return true;
				}
			}
		}
		else
		{
			std::cout << "Result " << srv.response.planning_time_ << std::endl;
			ROS_ERROR("Calling Exotica Planning service failed");
		}
		res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
		return false;
	}

	/** \brief Solve the motion planning problem and store the detailed result in \e res. This function should not clear data structures before computing. The constructor and clear() do that. */
	bool EXOTicaPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
	{
		planning_interface::MotionPlanResponse res2;
		const moveit::core::JointModelGroup* model_group =
				planning_scene_->getRobotModel()->getJointModelGroup(request_.group_name);
		if (solve(res2))
		{
			res.trajectory_.reserve(1);
			res.trajectory_.resize(res.trajectory_.size() + 1);
			res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(planning_scene_->getRobotModel(), model_group->getName()));
			copySolution(solution_, res.trajectory_.back().get());
			res.description_.push_back("plan");
			res.processing_time_.push_back(res2.planning_time_);
			res.error_code_ = res2.error_code_;
			return true;
		}
		else
		{
			return false;
		}
	}

	void EXOTicaPlanningContext::copySolution(const Eigen::Ref<const Eigen::MatrixXd> & solution,
			robot_trajectory::RobotTrajectory* traj)
	{
		const moveit::core::JointModelGroup* model_group =
				planning_scene_->getRobotModel()->getJointModelGroup(request_.group_name);
		traj->clear();
		moveit::core::RobotState state = start_state_;
		for (int t = 0; t < solution.rows(); t++)
		{
			state.setJointGroupPositions(model_group, solution.row(t));
			state.update();
			traj->addSuffixWayPoint(state, tau_);
		}
	}

	/** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if solve() is not running (returns true).*/
	bool EXOTicaPlanningContext::terminate()
	{
		//TODO - make interruptible
		ROS_WARN_STREAM("Attempting to terminate");
	}

	/** \brief Clear the data structures used by the planner */
	void EXOTicaPlanningContext::clear()
	{
		sol.reset();
	}

} /* namespace exotica */

CLASS_LOADER_REGISTER_CLASS(exotica::EXOTicaPlannerManager, planning_interface::PlannerManager);
