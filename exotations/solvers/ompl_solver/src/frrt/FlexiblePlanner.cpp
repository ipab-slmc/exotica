/*
 * FRRTBase.cpp
 *
 *  Created on: 8 Jun 2015
 *      Author: yiming
 */

#include <frrt/FlexiblePlanner.h>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
namespace ompl
{
	namespace geometric
	{
		FlexiblePlanner::FlexiblePlanner(const base::SpaceInformationPtr &si,
				const std::string & name) :
				base::Planner(si, name), checkCnt_(0)
		{
			specs_.approximateSolutions = true;
			specs_.directed = true;
		}

		FlexiblePlanner::~FlexiblePlanner()
		{

		}

		bool FlexiblePlanner::setUpLocalPlanner(const std::string & xml_file,
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
			local_map_ =
					boost::static_pointer_cast<exotica::Identity>(prob->getTaskMaps().at("CSpaceMap"));
			if (!exotica::ok(ser_->getParam(ser_->getName() + "/GlobalTau", gTau_)))
			{
				INDICATE_FAILURE
				return false;
			}
			if (!exotica::ok(ser_->getParam(ser_->getName() + "/LocalTau", lTau_)))
			{
				INDICATE_FAILURE
				return false;
			}

			return true;
		}

		bool FlexiblePlanner::resetSceneAndGoal(const exotica::Scene_ptr & scene,
				const Eigen::VectorXd & goal)
		{
			checkCnt_ = 0;
			global_goal_.setZero(goal.size());
			global_goal_ = goal;
			if (!ok(local_solver_->getProblem()->setScene(scene->getPlanningScene())))
			{
				INDICATE_FAILURE
				return false;
			}
			return true;
		}

		exotica::EReturn FlexiblePlanner::localSolve(const Eigen::VectorXd & qs,
				Eigen::VectorXd & qg, Eigen::MatrixXd & solution)
		{

			int dim = (int) si_->getStateDimension();
			exotica::EReturn ret = local_solver_->SolveFullSolution(qs, solution);
			qg.resize(dim);
			qg = solution.row(solution.rows() - 1).transpose();
			checkCnt_ += (solution.rows() - 1);
			return ret;
		}
	}
}

