/*
 * ik_solver.cpp
 *
 *  Created on: 15 Jul 2014
 *      Author: yiming
 */

#include "ik_solver/ik_solver.h"
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}

REGISTER_MOTIONSOLVER_TYPE("IKsolver", exotica::IKsolver);

namespace exotica
{
	IKsolver::IKsolver() :
			size_(0)
	{
		//TODO
	}

	IKsolver::~IKsolver()
	{

	}

	EReturn IKsolver::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("MaxIt");
		server_->registerParam<std_msgs::Int64>(ns_, tmp_handle, maxit_);
		tmp_handle = handle.FirstChildElement("MaxStep");
		server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, maxstep_);
		//ERROR("IK Solver parameters update [maximum iteration]: " << maxit_->data);
		//INFO("IK Solver parameters update [maximum step velocity]: " << maxstep_);
		return SUCCESS;
	}

	EReturn IKsolver::specifyProblem(PlanningProblem_ptr pointer)
	{
		if (pointer->type().compare(std::string("exotica::IKProblem")) != 0)
		{
			ERROR("This IKsolver can't solve problem of type '" << pointer->type() << "'!");
			return PAR_INV;
		}
		problem_ = pointer;
		prob_ = boost::static_pointer_cast<IKProblem>(pointer);
		size_ = prob_->getW().rows();

		for (auto & it : prob_->getTaskDefinitions())
		{
			if (it.second->type().compare(std::string("exotica::TaskSqrError")) != 0)
			{
				ERROR("IK Solver currently can only solve exotica::TaskSqrError. Unable to solve Task: "<<it.second->type());
				return FAILURE;
			}
		}
		return SUCCESS;
	}

	EReturn IKsolver::setGoal(const std::string & task_name, const Eigen::VectorXd & goal)
	{
//Lets now just consider one task at a time
		if (prob_->getTaskDefinitions().find(task_name) == prob_->getTaskDefinitions().end())
		{
			std::cout << "Task name " << task_name << " does not exist" << std::endl;
			return FAILURE;
		}

		boost::shared_ptr<TaskSqrError> task =
				boost::static_pointer_cast<TaskSqrError>(prob_->getTaskDefinitions().at(task_name));
		if (!ok(task->setGoal(goal)))
		{
			std::cout << "Setting goal for " << task_name << " failed" << std::endl;
			return FAILURE;
		}
		return SUCCESS;
	}

	IKProblem_ptr& IKsolver::getProblem()
	{
		return prob_;
	}

	int IKsolver::getMaxIteration()
	{
		return (int) maxit_->data;
	}

	EReturn IKsolver::setRho(const std::string & task_name, const double rho)
	{
		if (prob_->getTaskDefinitions().find(task_name) == prob_->getTaskDefinitions().end())
		{
			ERROR("Task name "<<task_name<<" does not exist");
			return FAILURE;
		}
		boost::shared_ptr<TaskSqrError> task =
				boost::static_pointer_cast<TaskSqrError>(prob_->getTaskDefinitions().at(task_name));
		if (!ok(task->setRho(rho)))
			return FAILURE;
		return SUCCESS;
	}
	EReturn IKsolver::Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution)
	{
		ros::Time start = ros::Time::now();
		ros::Duration t;
		if (size_ != q0.rows())
		{
			std::cout << "Wrong size q0 size=" << q0.rows() << ", required size=" << size_ << std::endl;
			INDICATE_FAILURE
			return FAILURE;
		}
		bool found = false;
		solution.resize(maxit_->data + 1, size_);
		solution.row(0) = q0;
		double err = 100, ini_err;
		Eigen::VectorXd q_out = q0;
		int i = 0;
		ros::Duration dt_update;
		ros::Duration dt_vel;
		double ii = 0;
		for (i = 0; i < maxit_->data; i++)
		{
			ii += 1.0;
			ros::Time t0 = ros::Time::now();
			prob_->update(solution.row(i), 0);
			dt_update += ros::Duration(ros::Time::now() - t0);
			t0 = ros::Time::now();
			vel_solve(err);
			dt_vel += ros::Duration(ros::Time::now() - t0);
			if (i == 0)
				ini_err = err;
			double max_vel = vel_vec_.cwiseAbs().maxCoeff();
			if (max_vel > maxstep_->data)
			{
				max_vel = maxstep_->data / max_vel;
				vel_vec_ *= max_vel;
			}
			solution.row(i + 1) = solution.row(i) + vel_vec_.transpose();

			if (err <= prob_->getTau())
			{
				solution.conservativeResize(i + 1, solution.cols());
				t = ros::Duration(ros::Time::now() - start);
				found = true;
				break;
			}
		}
		//ROS_WARN_STREAM_THROTTLE(0.5,"Update: "<< dt_update <<"s Solve: " << dt_vel<<"s");
		t = ros::Duration(ros::Time::now() - start);
		//std::cout << "IK solving time (" << i << " iterations) = " << t.toSec() << " sec\n";
		return SUCCESS;
	}

	EReturn IKsolver::vel_solve(double & err)
	{
		vel_vec_.resize(size_);
		vel_vec_.setZero();
		null_space_map = Eigen::MatrixXd::Identity(size_, size_);
		int dim = 0, big_size = 0, cnt = 0, cur_rows = 0;
		double rho;
		rhos.resize(prob_->getTaskDefinitions().size());
// Get big jacobian size and rho
		for (auto & it : prob_->getTaskDefinitions())
		{
			boost::shared_ptr<TaskSqrError> task =
					boost::static_pointer_cast<TaskSqrError>(it.second);
			task->getRho(rho);
			if (rho > 0)
			{
				task->taskSpaceDim(dim);
				big_size += dim;
				rhos[cnt] = rho * Eigen::VectorXd::Ones(dim);
				cnt++;
			}
		}
		cnt = 0;
		big_jacobian.resize(big_size, size_);
		task_weights.resize(big_size, big_size);
		task_weights.setZero();
		diag.resize(big_size);
		err = 0;
		cur_rows = 0;
// Get big task error
		task_error.resize(big_size, 1);

		// Get big jacobian and C
		for (auto & it : prob_->getTaskDefinitions())
		{
			boost::shared_ptr<TaskSqrError> task =
					boost::static_pointer_cast<TaskSqrError>(it.second);
			task->getRho(rho);
			if (rho > 0)
			{
				task->taskSpaceDim(dim);
				//Eigen::MatrixXd tmp_jac(dim, size_);
				task->jacobian(big_jacobian.block(cur_rows, 0, dim, size_));
				//big_jacobian.block(cur_rows, 0, dim, size_) = tmp_jac;
				diag.block(cur_rows, 0, dim, 1) = rhos[cnt];
				cnt++;

				goal.resize(dim);
				phi.resize(dim);
				if (!ok(task->getGoal(goal)))
				{
					std::cout << "Velocity solver get goal failed" << std::endl;
					return FAILURE;
				}
				if (!ok(task->phi(phi)))
				{
					std::cout << "Velocity solver get phi failed" << std::endl;
					return FAILURE;
				}
				task_error.segment(cur_rows, dim) = goal - phi;
				err += task_error.norm();
				cur_rows += dim;
			}
		}
		task_weights.diagonal() = diag;
		std::cout << "Big Jac\n" << big_jacobian << std::endl;
// Compute velocity

		inv_jacobian =
				((big_jacobian.transpose() * task_weights * big_jacobian + prob_->getW()).inverse()
						* big_jacobian.transpose() * task_weights); //(Jt*C*J+W)
		vel_vec_ = inv_jacobian * task_error;
		//null_space_map *= (Eigen::MatrixXd::Identity(size_, size_)- inv_jacobian * big_jacobian);
		return SUCCESS;
	}
}

