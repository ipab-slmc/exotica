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
			tmp_handle = handle.FirstChildElement("LocalMinimaThreshold");
			server_->registerParam<std_msgs::Float64>(ns_, tmp_handle, local_minima_threshold_);
			tmp_handle = handle.FirstChildElement("MultiTaskMode");
			server_->registerParam<std_msgs::Bool>(ns_, tmp_handle, multi_task_);

			jac_pub_ = server_->advertise<visualization_msgs::MarkerArray>("JacobianVector", 1);
			jac_arr_.markers.resize(2);
			jac_arr_.markers[0].type = visualization_msgs::Marker::ARROW;
			jac_arr_.markers[1].type = visualization_msgs::Marker::ARROW;
			jac_arr_.markers[0].header.frame_id = "base";
			jac_arr_.markers[1].header.frame_id = "base";
			jac_arr_.markers[0].id = 0;
			jac_arr_.markers[1].id = 1;
			jac_arr_.markers[0].color.r = 1;
			jac_arr_.markers[0].color.a = 1;
			jac_arr_.markers[1].color.g = 1;
			jac_arr_.markers[1].color.a = 1;

			jac_arr_.markers[0].scale.x = 0.015;
			jac_arr_.markers[0].scale.y = 0.04;
			jac_arr_.markers[0].scale.z = 0.04;
			jac_arr_.markers[1].scale.x = 0.015;
			jac_arr_.markers[1].scale.y = 0.04;
			jac_arr_.markers[1].scale.z = 0.04;
		}
		catch (int e)
		{
			std::cout << "IK solver initialisation, parameter error\n";
			return FAILURE;
		}
		HIGHLIGHT_NAMED(object_name_, "Initialised at "<<this);
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
			if (!ok(it.second->activateTaskMaps()))
			{
				INDICATE_FAILURE
				;
				return FAILURE;
			}
		}

		T = prob_->getT();
		int big_size = 0;
		int i = 0, cur_rows = 0;

		rhos.resize(T);
		big_jacobian.resize(T);
		goal.resize(T);
		phi.resize(T);
		dim.resize(T);
        _rhos.resize(T);
        _jacobian.resize(T);
        _goal.resize(T);
        _phi.resize(T);
		taskIndex.clear();

		for (int t = 0; t < T; t++)
		{
			dim.at(t).resize(prob_->getTaskDefinitions().size());
			i = 0;
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
			weights.resize(prob_->getTaskDefinitions().size());
			phi.at(t).resize(big_size);
			goal.at(t).resize(big_size);
			task_error.resize(big_size, 1);
			rhos.at(t).resize(prob_->getTaskDefinitions().size());
			rhos.at(t).setZero();
			goal.at(t).setZero();

            _rhos[t].resize(prob_->getTaskDefinitions().size());
            _jacobian[t].resize(prob_->getTaskDefinitions().size());
            _goal[t].resize(prob_->getTaskDefinitions().size());
            _phi[t].resize(prob_->getTaskDefinitions().size());
			i = 0;
			for (auto & it : prob_->getTaskDefinitions())
			{
				TaskSqrError_ptr task = boost::static_pointer_cast<TaskSqrError>(it.second);
                _rhos[t][i]=Eigen::VectorXdRef_ptr(rhos.at(t).segment(i, 1));
                _goal[t][i]=Eigen::VectorXdRef_ptr(goal.at(t).segment(cur_rows, dim.at(t)(i)));
                _phi[t][i]=Eigen::VectorXdRef_ptr(phi.at(t).segment(cur_rows, dim.at(t)(i)));
                _jacobian[t][i]=Eigen::MatrixXdRef_ptr(big_jacobian.at(t).block(cur_rows, 0, dim.at(t)(i), size_));

                task->registerRho(_rhos[t][i], t);
                task->registerGoal(_goal[t][i], t);
				task->setDefaultGoals(t);
                task->registerPhi(_phi[t][i], t);
                task->registerJacobian(_jacobian[t][i], t);

				task_weights.diagonal().block(cur_rows, 0, dim.at(t)(i), 1).setConstant(rhos.at(t)(i));
				weights[i].resize(dim.at(t)(i), dim.at(t)(i));
				weights[i].diagonal() = task_weights.diagonal().block(cur_rows, 0, dim.at(t)(i), 1);
				if (t == 0)
				{
					taskIndex[it.first] = std::pair<int, int>(i, cur_rows);
				}
				cur_rows += dim.at(t)(i);
				i++;
			}
		}
		tasks_ = prob_->getTaskDefinitions();
		JTCinv_.resize(tasks_.size());
		JTCinvJ_.resize(size_, size_);
        JTCinvdy_.resize(size_);
		initialised_ = true;
		return SUCCESS;
	}

	bool IKsolver::isSolvable(const PlanningProblem_ptr & prob)
	{
		if (prob->type().compare("exotica::IKProblem") == 0)
			return true;
		return false;
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
			std::pair<int, int> id = taskIndex.at(task_name);
			if (_goal.rows() == dim.at(t)(id.first))
			{
				goal.at(t).segment(id.second, dim.at(t)(id.first)) = _goal;
				return SUCCESS;
			}
			else
			{
				INDICATE_FAILURE
				;
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
			std::pair<int, int> id = taskIndex.at(task_name);
			rhos.at(t)(id.first) = rho;
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
		if (initialised_)
		{
			ros::Time start = ros::Time::now();
			if (size_ != q0.rows())
			{
				std::cout << "Wrong size q0 size=" << q0.rows() << ", required size=" << size_
						<< std::endl;
				INDICATE_FAILURE
				return FAILURE;
			}
			solution.resize(T, size_);

			Eigen::VectorXd q = q0;
			for (int t = 0; t < T; t++)
			{

				if (ok(Solve(q, solution.block(t, 0, 1, size_), t)))
				{
					q = solution.row(t);
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
			INDICATE_FAILURE
			;
			return FAILURE;
		}
	}

	EReturn IKsolver::Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXdRef solution, int t)
	{
		if (initialised_)
		{
			vel_vec_.resize(size_);

			solution.row(0) = q0;
			error = INFINITY;
			bool found = false;
			maxdim_ = 0;
			for (int i = 0; i < maxit_->data; i++)
			{
				if (ok(prob_->update(solution.row(0), t)))
				{
                    if(!ok(vel_solve(error, t, solution.row(0))))
                    {
                        INDICATE_FAILURE
                        return FAILURE;
                    }
					double max_vel = vel_vec_.cwiseAbs().maxCoeff();
					if (max_vel > maxstep_->data)
					{
						vel_vec_ = vel_vec_ * maxstep_->data / max_vel;
					}
                    if(error <= prob_->getTau()*2.0)
                    {
                        solution.row(0) = solution.row(0) + vel_vec_.transpose();
                    }
                    else
                    {
                        solution.row(0) = solution.row(0) + vel_vec_.transpose()*0.5;
                    }
					if (error <= prob_->getTau())
					{
						found = true;
						break;
					}
				}
				else
				{
					INDICATE_FAILURE
					;
					return FAILURE;
				}
			}

			if (found)
			{
				return SUCCESS;
			}
			else
			{
				WARNING("Solution not found after reaching max number of iterations");
				return WARNING;
			}
		}
		else
		{
			INDICATE_FAILURE
			;
			return FAILURE;
		}
	}

	EReturn IKsolver::SolveFullSolution(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution)
	{
		int t = 0;
		if (initialised_)
		{
			ros::Time start = ros::Time::now();
			vel_vec_.resize(size_);
			Eigen::MatrixXd tmp(maxit_->data, size_);
			solution.resize(1, size_);
			solution.row(0) = q0;
			tmp.row(0) = q0;
			error = INFINITY;
			bool found = false;
			maxdim_ = 0;
			int i = 0;
			for (i = 0; i < maxit_->data; i++)
			{
				if (ok(prob_->update(solution.row(0), t)))
				{

                    if(!ok(vel_solve(error, t, solution.row(0))))
                    {
                        INDICATE_FAILURE
                        return FAILURE;
                    }
					if (vel_vec_ != vel_vec_)
					{
						INDICATE_FAILURE
						return FAILURE;
					}
					double max_vel = vel_vec_.cwiseAbs().maxCoeff();
					if (max_vel > maxstep_->data)
					{
						vel_vec_ = vel_vec_ * maxstep_->data / max_vel;
					}
					solution.row(0) = solution.row(0) + vel_vec_.transpose();
					tmp.row(i + 1) = solution.row(0);
					static double init_err = error;
					if (error <= prob_->getTau())
					{
						found = true;
						break;
					}
					else if (i > 2 && error > 0.1 * init_err)
					{
						double change = (tmp.row(i + 1) - tmp.row(i - 1)).cwiseAbs().sum();
						if (change < maxstep_->data)
						{
							WARNING_NAMED(object_name_, "Running into local minima with velocity "<<change);
							return FAILURE;
						}
					}
				}
				else
				{
					return FAILURE;
				}
//				std::cout << "BIG "<<i<<" :\n" << task_weights * big_jacobian.at(t) << std::endl;
//				std::cout << "Phi: "<<phi.at(t).transpose() << std::endl;
//				getchar();
			}

			if (found)
			{
				solution.resize(i + 1, size_);
				solution = tmp.block(0, 0, i + 1, size_);
				INFO_NAMED(object_name_, "IK solution found in "<<ros::Duration(ros::Time::now()-start).toSec()<<"sec");
				return SUCCESS;
			}
			else
			{
				WARNING_NAMED(object_name_, "Solution not found after reaching max number of iterations, with error "<<error<<" ("<<ros::Duration(ros::Time::now()-start).toSec()<<"sec) Tolerance: "<<prob_->getTau());
				return FAILURE;
			}
		}
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}
	}

    template<typename Scalar>
    struct CwiseClampOp {
      CwiseClampOp(const Scalar& inf, const Scalar& sup) : m_inf(inf), m_sup(sup) {}
      const Scalar operator()(const Scalar& x) const { return x<m_inf ? m_inf : (x>m_sup ? m_sup : x); }
      Scalar m_inf, m_sup;
    };

	EReturn IKsolver::vel_solve(double & err, int t, Eigen::VectorXdRefConst q)
	{
		static Eigen::MatrixXd I =
				Eigen::MatrixXd::Identity(prob_->getW().rows(), prob_->getW().rows());
		if (initialised_)
		{
			vel_vec_.setZero();
			task_error = goal.at(t) - phi.at(t);
			err = (task_weights * task_error).squaredNorm();

			if (!multi_task_->data)
			{
				// Compute velocity
				Eigen::MatrixXd Jpinv;
                big_jacobian.at(t)=big_jacobian.at(t).unaryExpr(CwiseClampOp<double>(-1e10,1e10));
                Jpinv = (big_jacobian.at(t).transpose() * task_weights * big_jacobian.at(t)
                        + prob_->getW()).inverse() * big_jacobian.at(t).transpose() * task_weights; //(Jt*C*J+W)*Jt*C */
                /*Jpinv = prob_->getW()*big_jacobian.at(t).transpose() * (big_jacobian.at(t).transpose()*prob_->getW()*big_jacobian.at(t) + Eigen::MatrixXd::Identity(task_weights.rows(),task_weights.rows())*task_weights ).inverse(); //W*Jt*(Jt*W*J+C)*/
                /*Jpinv=big_jacobian.at(t).transpose();*/
				if (Jpinv != Jpinv)
				{
					INDICATE_FAILURE
                    HIGHLIGHT(big_jacobian.at(t));
                    return FAILURE;
				}
                /*if(nullSpaceRef.rows()==q.rows())
                {
                    vel_vec_ = Jpinv * task_error + (I-Jpinv*big_jacobian.at(t))*(nullSpaceRef-q);
                }
                else*/
                {
                    vel_vec_ = Jpinv * task_error;
                }

			}
			else
			{
				int i = 0;
				JTCinvJ_.setZero();
				JTCinvdy_.setZero();
				for (auto & it : tasks_)
				{
                    JTCinv_[i] = _jacobian[t][i]->transpose() * weights[i];
                    JTCinvJ_ += JTCinv_[i] * (*(_jacobian[t][i])) + prob_->getW();
                    JTCinvdy_ += JTCinv_[i] * ((*(_goal[t][i]))-(*(_phi[t][i])));
					i++;
				}
//				jac_pub_.publish(jac_arr_);
                vel_vec_ = JTCinvJ_.inverse() * JTCinvdy_;
			}
			return SUCCESS;
		}
		else
		{
			INDICATE_FAILURE
			;
			return FAILURE;
		}
	}
}

