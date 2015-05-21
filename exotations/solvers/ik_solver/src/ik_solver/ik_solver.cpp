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
            size_(0), T(0), initialised_(false)
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
        MotionSolver::specifyProblem(pointer);
		prob_ = boost::static_pointer_cast<IKProblem>(pointer);
        size_ = prob_->getScenes().begin()->second->getNumJoints();
        for (auto & it : prob_->getScenes())
        {
            if(!ok(it.second->activateTaskMaps()))
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }

        T=prob_->getT();
        int big_size = 0;
        int i=0, cur_rows=0;

        rhos.resize(T);
        big_jacobian.resize(T);
        goal.resize(T);
        phi.resize(T);
        dim.resize(T);
        taskIndex.clear();

        for(int t=0;t<T;t++)
        {
            dim.at(t).resize(prob_->getTaskDefinitions().size());
            i=0;
            for (auto & it : prob_->getTaskDefinitions())
            {
                if (it.second->type().compare(std::string("exotica::TaskSqrError")) != 0)
                {
                    ERROR("IK Solver currently can only solve exotica::TaskSqrError. Unable to solve Task: "<<it.second->type());
                    return FAILURE;
                }
                else
                {
                    TaskSqrError_ptr task = boost::static_pointer_cast<TaskSqrError>(it.second);
                    task->taskSpaceDim(dim.at(t)(i));
                }
                i++;
            }
            big_size = dim.at(t).sum();

            big_jacobian.at(t).resize(big_size, size_);
            task_weights.resize(big_size);
            phi.at(t).resize(big_size);
            goal.at(t).resize(big_size);
            task_error.resize(big_size, 1);
            rhos.at(t).resize(prob_->getTaskDefinitions().size());
            rhos.at(t).setZero();
            goal.at(t).setZero();
            i=0;
            for (auto & it : prob_->getTaskDefinitions())
            {
                TaskSqrError_ptr task = boost::static_pointer_cast<TaskSqrError>(it.second);
                task->registerRho(Eigen::VectorXdRef_ptr(rhos.at(t).segment(i,1)),t);
                task->registerGoal(Eigen::VectorXdRef_ptr(goal.at(t).segment(cur_rows,dim.at(t)(i))),t);
                task->setDefaultGoals(t);
                task->registerPhi(Eigen::VectorXdRef_ptr(phi.at(t).segment(cur_rows,dim.at(t)(i))),t);
                task->registerJacobian(Eigen::MatrixXdRef_ptr(big_jacobian.at(t).block(cur_rows,0,dim.at(t)(i),size_)),t);
                task_weights.diagonal().block(cur_rows, 0, dim.at(t)(i), 1).setConstant(rhos.at(t)(i));
                if(t==0)
                {
                    taskIndex[it.first]= std::pair<int,int>(i,cur_rows);
                }
                cur_rows += dim.at(t)(i);
                i++;
            }
        }
        initialised_=true;
		return SUCCESS;
	}

    EReturn IKsolver::setGoal(const std::string & task_name, Eigen::VectorXdRefConst _goal, int t)
	{
        if (taskIndex.find(task_name) == taskIndex.end())
		{
			std::cout << "Task name " << task_name << " does not exist" << std::endl;
			return FAILURE;
		}
        else
        {
            std::pair<int,int> id = taskIndex.at(task_name);
            if(_goal.rows()==dim.at(t)(id.first))
            {
                goal.at(t).segment(id.second,dim.at(t)(id.first))=_goal;
                return SUCCESS;
            }
            else
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }
	}

    EReturn IKsolver::setRho(const std::string & task_name, const double rho, int t)
    {
        if (taskIndex.find(task_name) == taskIndex.end())
        {
            std::cout << "Task name " << task_name << " does not exist" << std::endl;
            return FAILURE;
        }
        else
        {
            std::pair<int,int> id = taskIndex.at(task_name);
            rhos.at(t)(id.first)=rho;
            return SUCCESS;
        }
    }

	IKProblem_ptr& IKsolver::getProblem()
	{
		return prob_;
	}

	int IKsolver::getMaxIteration()
	{
		return (int) maxit_->data;
	}


    EReturn IKsolver::Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution)
    {
        if(initialised_)
        {
            ros::Time start = ros::Time::now();
            if (size_ != q0.rows())
            {
                std::cout << "Wrong size q0 size=" << q0.rows() << ", required size=" << size_ << std::endl;
                INDICATE_FAILURE
                return FAILURE;
            }
            solution.resize(T, size_);

            Eigen::VectorXd q = q0;
            for(int t=0;t<T;t++)
            {
                if(ok(Solve(q,solution.block(t,0,1,size_),t)))
                {
                    q=solution.row(t);
                }
                else
                {
                    INDICATE_FAILURE;
                    return FAILURE;
                }
            }
            planning_time_ = ros::Duration(ros::Time::now() - start);
            return SUCCESS;
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
    }

    EReturn IKsolver::Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXdRef solution, int t)
	{
        if(initialised_)
        {
            vel_vec_.resize(size_);

            solution.row(0) = q0;
            error = INFINITY;
            bool found = false;
            maxdim_=0;
            for (int i = 0; i < maxit_->data; i++)
            {
                if(ok(prob_->update(solution.row(0), t)))
                {
                    vel_solve(error,t,solution.row(0));
                    double max_vel = vel_vec_.cwiseAbs().maxCoeff();
                    if(max_vel > maxstep_->data)
                    {
                        vel_vec_ = vel_vec_*maxstep_->data / max_vel;
                    }

                    solution.row(0) = solution.row(0) + vel_vec_.transpose();

                    if (error <= prob_->getTau())
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



            if(found)
            {
                return SUCCESS;
            }
            else
            {
                INFO("Solution not found after reaching max number of iterations");
                return WARNING;
            }
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
	}

    EReturn IKsolver::vel_solve(double & err, int t, Eigen::VectorXdRefConst q)
    {
        static Eigen::MatrixXd I = Eigen::MatrixXd::Identity(prob_->getW().rows(),prob_->getW().rows());
        if(initialised_)
        {
            vel_vec_.setZero();
            int cur_rows = 0;
            for (int i=0;i<dim.at(t).rows();i++)
            {
                task_weights.diagonal().block(cur_rows, 0, dim.at(t)(i), 1).setConstant(rhos.at(t)(i));
                cur_rows += dim.at(t)(i);
            }


            task_error = goal.at(t) - phi.at(t);
            err = (task_weights*task_error).squaredNorm();
            // Compute velocity
            Eigen::MatrixXd Jpinv;
            Jpinv = (big_jacobian.at(t).transpose() * task_weights * big_jacobian.at(t) + prob_->getW() ).inverse()
                    * big_jacobian.at(t).transpose() * task_weights; //(Jt*C*J+W)*Jt*C

            vel_vec_ = Jpinv* task_error;
            return SUCCESS;


        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
	}
}

