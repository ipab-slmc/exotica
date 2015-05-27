/*
 * EXOTicaPlannerService.cpp
 *
 *  Created on: 19 Mar 2015
 *      Author: yiming
 */

#include "exotica_moveit/EXOTicaPlannerService.h"

namespace exotica
{
	EXOTicaPlannerService::EXOTicaPlannerService() :
					as_(nh_, "/ExoticaPlanning", boost::bind(&exotica::EXOTicaPlannerService::solve, this, _1), false),
					initialised_(false)
	{

	}

	EXOTicaPlannerService::~EXOTicaPlannerService()
	{

	}

	bool EXOTicaPlannerService::initialise(const std::string & config, const std::string & solver,
			const std::string & problem, const std::string & group)
	{
		exotica::Initialiser ini;

		INFO("Loading from "<<config);
		if (!ok(ini.initialise(config, server_, solver_, problem_, problem, solver)))
		{
			ERROR("EXOTica/MoveIt Action service: EXOTica initialisation failed !!!!");
			initialised_ = false;
		}
		else
		{
			if (solver_->type().compare("exotica::AICOsolver") == 0)
			{
				;
			}
			else if (solver_->type().compare("exotica::IKsolver") == 0)
			{
				;
			}
			else if (solver_->type().compare("exotica::OMPLsolver") == 0)
			{
				const moveit::core::JointModelGroup* model_group =
						server_->getModel("robot_description")->getJointModelGroup(group);
				moveit::core::JointBoundsVector b = model_group->getActiveJointModelsBounds();
				exotica::OMPLProblem_ptr tmp =
						boost::static_pointer_cast<exotica::OMPLProblem>(problem_);
				tmp->getBounds().resize(b.size() * 2);
				for (int i = 0; i < b.size(); i++)
				{
					tmp->getBounds()[i] = (*b[i])[0].min_position_;
					tmp->getBounds()[i + b.size()] = (*b[i])[0].max_position_;
				}
			}

			if (!exotica::ok(solver_->specifyProblem(problem_)))
			{
				INDICATE_FAILURE
				initialised_ = false;
			}
			else
			{
				initialised_ = true;
				as_.start();
			}
		}
		return initialised_;
	}
	bool EXOTicaPlannerService::solve(const exotica_moveit::ExoticaPlanningGoalConstPtr & goal)
	{
		res_.succeeded_ = false;
		scene_.reset(new moveit_msgs::PlanningScene(goal->scene_));
		if (!ok(problem_->setScene(scene_)))
		{
			INDICATE_FAILURE
			return false;
		}
//		HIGHLIGHT_NAMED("MoveitInterface", "Using Solver "<<solver_->object_name_<<"["<<solver_->type()<<"], Problem "<<problem_->object_name_<<"["<<problem_->type()<<"].");
		if (solver_->type().compare("exotica::OMPLsolver") == 0)
		{
			exotica::OMPLsolver_ptr ss = boost::static_pointer_cast<exotica::OMPLsolver>(solver_);

			Eigen::VectorXd qT;
			exotica::vectorExoticaToEigen(goal->qT, qT);
			boost::static_pointer_cast<exotica::Identity>(ss->getProblem()->getTaskMaps().at("CSpaceGoalMap"))->jointRef=qT;
			ss->setMaxPlanningTime(goal->max_time_);
			if (!ok(ss->resetIfNeeded()))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
		}

		Eigen::VectorXd q0;
		Eigen::MatrixXd solution;
		exotica::vectorExoticaToEigen(goal->q0, q0);
		ros::Time start = ros::Time::now();
		EReturn found = FAILURE;
		if (solver_->type().compare("exotica::IKsolver") == 0)
		{
			found =
					boost::static_pointer_cast<exotica::IKsolver>(solver_)->SolveFullSolution(q0, solution);
		}
		else
			found = solver_->Solve(q0, solution);

		if (ok(found))
		{
			res_.succeeded_ = true;
			fb_.solving_time_ = res_.planning_time_ =
					ros::Duration(ros::Time::now() - start).toSec();
			exotica::matrixEigenToExotica(solution, res_.solution_);
			as_.setSucceeded(res_);
		}
		return res_.succeeded_;
	}
}

