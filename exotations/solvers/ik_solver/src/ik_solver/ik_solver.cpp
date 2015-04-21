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
		try
		{
			tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("MaxIt");
			server_->registerParam<std_msgs::Int64>(ns_, tmp_handle, maxit_);
			tmp_handle = handle.FirstChildElement("MaxStep");
			server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, maxstep_);
		}
		catch (int e)
		{
			std::cout << "IK solver initialisation, parameter error\n";
			return FAILURE;
		}

		//INFO("IK Solver parameters update [maximum iteration]: " << maxit_);
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
        Solve(q0,solution,0);
    }

    EReturn IKsolver::Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution, int t)
	{
		ros::Time start = ros::Time::now();

		if (size_ != q0.rows())
		{
			std::cout << "Wrong size q0 size=" << q0.rows() << ", required size=" << size_ << std::endl;
			INDICATE_FAILURE
			return FAILURE;
		}
        solution.resize(1, size_);
        vel_vec_.resize(size_);
        rhos.resize(prob_->getTaskDefinitions().size());
		solution.row(0) = q0;
        double err = INFINITY;
        bool found = false;
        maxdim_=0;
        for (int i = 0; i < maxit_->data; i++)
		{
            if(ok(prob_->update(solution.row(0), t)))
            {
                vel_solve(err,t);
                double max_vel = vel_vec_.cwiseAbs().maxCoeff();
                vel_vec_ = max_vel > maxstep_->data?vel_vec_*maxstep_->data / max_vel:vel_vec_;

                solution.row(0) = solution.row(0) + vel_vec_.transpose();

                if (err <= prob_->getTau())
                {
                    found = true;
                    break;
                }
            }
            else
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
		}

        planning_time_ = ros::Duration(ros::Time::now() - start);

        if(found)
        {
            return SUCCESS;
        }
        else
        {
            return FAILURE;
        }
	}

    EReturn IKsolver::vel_solve(double & err, int t)
	{
        vel_vec_.setZero();
		int dim = 0, big_size = 0, cnt = 0, cur_rows = 0;
		double rho;

// Get big jacobian size and rho
        for (auto & it : prob_->getTaskDefinitions())
		{
            boost::shared_ptr<TaskSqrError> task = boost::static_pointer_cast<TaskSqrError>(it.second);
            task->getRho(rho, t);
			if (rho > 0)
			{
				task->taskSpaceDim(dim);
				big_size += dim;
                maxdim_=dim>maxdim_?dim:maxdim_;
				rhos[cnt] = rho * Eigen::VectorXd::Ones(dim);
				cnt++;
			}
		}
		cnt = 0;
		big_jacobian.resize(big_size, size_);
        task_weights.resize(big_size);
        goal.resize(maxdim_);
        phi.resize(maxdim_);

		err = 0;
		cur_rows = 0;
        // Get big task error
		task_error.resize(big_size, 1);

		// Get big jacobian and C
		for (auto & it : prob_->getTaskDefinitions())
		{
			boost::shared_ptr<TaskSqrError> task =
					boost::static_pointer_cast<TaskSqrError>(it.second);
            task->getRho(rho,t);
			if (rho > 0)
			{
                task->taskSpaceDim(dim);
                task->jacobian(big_jacobian.block(cur_rows, 0, dim, size_),t);
                task_weights.diagonal().block(cur_rows, 0, dim, 1) = rhos[cnt];
				cnt++;


                if (!ok(task->getGoal(goal.head(dim),t)))
				{
					std::cout << "Velocity solver get goal failed" << std::endl;
					return FAILURE;
				}
                if (!ok(task->phi(phi.head(dim),t)))
				{
					std::cout << "Velocity solver get phi failed" << std::endl;
					return FAILURE;
				}
                task_error.segment(cur_rows, dim) = goal.head(dim) - phi.head(dim);
				err += task_error.norm();
				cur_rows += dim;
			}
        }

        // Compute velocity

        vel_vec_ =
                (((big_jacobian.transpose() * task_weights * big_jacobian + prob_->getW()).inverse()
                * big_jacobian.transpose() * task_weights))* task_error; //(Jt*C*J+W)
        return SUCCESS;
	}
}

