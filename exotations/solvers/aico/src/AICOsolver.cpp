/*
 *  Created on: 19 Apr 2014
 *      Author: Vladimir Ivan
 *
 *  This code is based on algorithm developed by Marc Toussaint
 *  M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
 *  http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
 *  Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */


/** \file AICOsolver.h
 \brief Approximate Inference Control */

#include "aico/AICOsolver.h"

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
#define MAT_OK(x) if(!ok(x)){INDICATE_FAILURE; exit(1);}
// t_0 is the start state
// t_T is the final state
// t_{T+1} is the state before t_0 for computing the velocity at the time t_0
#define TT T+2

REGISTER_MOTIONSOLVER_TYPE("AICOsolver", exotica::AICOsolver);

namespace exotica
{
  EReturn AICOsolver::saveCosts(std::string file_name)
  {
    std::ofstream myfile;
    myfile.open(file_name.c_str());
    myfile << "Control";
    for (auto s : taskNames)
      myfile << " " << s;
    for (int t = 0; t < T + 1; t++)
    {
      myfile << "\n" << costControl(t);
      for (int i = 0; i < costTask.cols(); i++)
      {
        myfile << " " << costTask(t, i);
      }
    }
    myfile.close();
    return SUCCESS;
  }

  AICOsolver::AICOsolver()
      : damping(0.01), tolerance(1e-2), max_iterations(100), useBwdMsg(false), bwdMsg_v(), bwdMsg_Vinv(), phiBar(), JBar(), s(), Sinv(), v(), Vinv(), r(), R(), rhat(), b(), Binv(), q(), qhat(), s_old(), Sinv_old(), v_old(), Vinv_old(), r_old(), R_old(), rhat_old(), b_old(), Binv_old(), q_old(), qhat_old(), dampingReference(), cost(
          0.0), cost_old(0.0), b_step(0.0), A(), tA(), Ainv(), invtA(), a(), B(), tB(), Winv(), Hinv(), Q(), sweep(
          0), sweepMode(0), W(), H(), T(0), dynamic(false), n(0), updateCount(
          0), damping_init(0.0), preupdateTrajectory_(false), q_stat()
  {

  }

  EReturn AICOsolver::getStats(std::vector<SinglePassMeanCoviariance>& q_stat_)
  {
    q_stat_ = q_stat;
    return SUCCESS;
  }

  AICOsolver::~AICOsolver()
  {
    // If this is not empty, your code is bad and you should feel bad!
    // Whoop whoop whoop whoop ...
  }

  EReturn AICOsolver::setGoal(const std::string & task_name,
      Eigen::VectorXdRefConst _goal, int t)
  {
    if (taskIndex.find(task_name) == taskIndex.end())
    {
      std::cout << "Task name " << task_name << " does not exist" << std::endl;
      return FAILURE;
    }
    else
    {
      std::pair<int, int> id = taskIndex.at(task_name);
      if (_goal.rows() == dim(id.first))
      {
        y_star.at(t).segment(id.second, dim(id.first)) = _goal;
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

  EReturn AICOsolver::setRho(const std::string & task_name, const double rho,
      int t)
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

  EReturn AICOsolver::getGoal(const std::string & task_name,
      Eigen::VectorXd& goal, int t)
  {
    if (taskIndex.find(task_name) == taskIndex.end())
    {
      std::cout << "Task name " << task_name << " does not exist" << std::endl;
      return FAILURE;
    }
    else
    {
      std::pair<int, int> id = taskIndex.at(task_name);
      goal = y_star.at(t).segment(id.second, dim(id.first));
      return SUCCESS;
    }
  }

  EReturn AICOsolver::getRho(const std::string & task_name, double& rho, int t)
  {
    if (taskIndex.find(task_name) == taskIndex.end())
    {
      std::cout << "Task name " << task_name << " does not exist" << std::endl;
      return FAILURE;
    }
    else
    {
      std::pair<int, int> id = taskIndex.at(task_name);
      rho = rhos.at(t)(id.first);
      return SUCCESS;
    }
  }

  EReturn AICOsolver::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLElement* xmltmp;
    XML_CHECK("sweepMode");
    const char* txt = xmltmp->GetText();
    if (strcmp(txt, "Forwardly") == 0)
      sweepMode = smForwardly;
    else if (strcmp(txt, "Symmetric") == 0)
      sweepMode = smSymmetric;
    else if (strcmp(txt, "LocalGaussNewton") == 0)
      sweepMode = smLocalGaussNewton;
    else if (strcmp(txt, "LocalGaussNewtonDamped") == 0)
      sweepMode = smLocalGaussNewtonDamped;
    else
    {
      INDICATE_FAILURE
      ;
      return PAR_ERR;
    }
    XML_CHECK("max_iterations");
    XML_OK(getInt(*xmltmp, max_iterations));
    XML_CHECK("tolerance");
    XML_OK(getDouble(*xmltmp, tolerance));
    XML_CHECK("damping");
    XML_OK(getDouble(*xmltmp, damping_init));
    int tmpi;
    XML_CHECK("UseBackwardMessage");
    XML_OK(getInt(*xmltmp, tmpi));
    useBwdMsg = tmpi != 0;
    XML_CHECK("dynamic");
    XML_OK(getInt(*xmltmp, tmpi));
    dynamic = tmpi != 0;
    return SUCCESS;
  }

  EReturn AICOsolver::specifyProblem(PlanningProblem_ptr pointer)
  {
    if (pointer->type().compare(std::string("exotica::AICOProblem")) != 0)
    {
      ERROR(
          "This solver can't use problem of type '" << pointer->type() << "'!");
      return PAR_INV;
    }
    MotionSolver::specifyProblem(pointer);
    prob_ = boost::static_pointer_cast<AICOProblem>(pointer);

    T = prob_->getT();
    taskNames.resize(prob_->getTaskDefinitions().size());
    taskIndex.clear();
    dim.resize(prob_->getTaskDefinitions().size());
    int i = 0, cur_rows = 0;
    for (auto& task_ : prob_->getTaskDefinitions())
    {
      if (task_.second->type().compare(std::string("TaskSqrError")) == 0)
      {
        ERROR("Task variable " << task_.first << " is not an squared error!");
        return FAILURE;
      }
      boost::shared_ptr<TaskSqrError> task = boost::static_pointer_cast<
          TaskSqrError>(task_.second);
      task->taskSpaceDim(dim(i));
      taskNames[i] = task_.first;
      taskIndex[task_.first] = std::pair<int, int>(i, cur_rows);
      cur_rows += dim(i);
      i++;
    }

    for (auto & it : prob_->getScenes())
    {
      if (!ok(it.second->activateTaskMaps()))
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }

    if (!ok(initMessages()))
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    return SUCCESS;
  }

  bool AICOsolver::isSolvable(const PlanningProblem_ptr & prob)
  {
    if (prob->type().compare("exotica::AICOProblem") == 0) return true;
    return false;
  }

  AICOProblem_ptr& AICOsolver::getProblem()
  {
    return prob_;
  }

  EReturn AICOsolver::Solve(Eigen::VectorXdRefConst q0,
      Eigen::MatrixXd & solution)
  {
    std::vector<Eigen::VectorXd> q_init;
    q_init.resize(TT, Eigen::VectorXd::Zero(q0.rows()));
    for (int i = 0; i < q_init.size(); i++)
      q_init[i] = q0;
    return Solve(q_init, solution);
  }

  EReturn AICOsolver::Solve(const std::vector<Eigen::VectorXd>& q_init,
      Eigen::MatrixXd & solution)
  {
    HIGHLIGHT_NAMED("AICO debug:\n", print(""))
    ros::Time startTime = ros::Time::now();
    ROS_WARN_STREAM("AICO: Setting up the solver");
    updateCount = 0;
    sweep = -1;
    damping = damping_init;
    double d;
    if (!(T > 0))
    {
      ERROR("Problem has not been initialized properly: T=0!");
      return FAILURE;
    }
    if (!ok(initTrajectory(q_init)))
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    sweep = 0;
    ROS_WARN_STREAM("AICO: Solving");
    for (int k = 0; k < max_iterations && ros::ok(); k++)
    {
      d = step();
      if (d < 0)
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
      if (k && d < tolerance) break;
    }
    Eigen::MatrixXd sol(T + 1, n);
    for (int tt = 0; tt <= T; tt++)
    {
      sol.row(tt) = q[tt];
    }
    solution = sol;
    planning_time_ = ros::Time::now() - startTime;
    return SUCCESS;
  }

  EReturn AICOsolver::initMessages()
  {
    if (prob_ == nullptr)
    {
      ERROR("Problem definition is a NULL pointer!");
      return FAILURE;
    }
    // TODO: Issue #4
    n = prob_->getScenes().begin()->second->getNumJoints();
    int n2 = n / 2;
    if (dynamic)
    {
      if (n < 2)
      {
        ERROR("State dimension is too small: n="<<n);
        return FAILURE;
      }
    }
    else
    {
      if (n < 1)
      {
        ERROR("State dimension is too small: n="<<n);
        return FAILURE;
      }
    }
    if (T < 2)
    {
      ERROR("Number of time steps is too small: T="<<T);
      return FAILURE;
    }

    s.assign(TT, Eigen::VectorXd::Zero(n));
    Sinv.assign(TT, Eigen::MatrixXd::Zero(n, n));
    Sinv[0].diagonal().setConstant(1e10);
    v.assign(TT, Eigen::VectorXd::Zero(n));
    Vinv.assign(TT, Eigen::MatrixXd::Zero(n, n));
    if (useBwdMsg)
    {
      if (bwdMsg_v.rows() == n && bwdMsg_Vinv.rows() == n
          && bwdMsg_Vinv.cols() == n)
      {
        v[T] = bwdMsg_v;
        Vinv[T] = bwdMsg_Vinv;
      }
      else
      {
        useBwdMsg = false;
        WARNING(
            "Backward message initialisation skipped, matrices have incorrect dimensions.");
      }
    }
    b.assign(TT, Eigen::VectorXd::Zero(n));
    dampingReference.assign(TT, Eigen::VectorXd::Zero(n));
    Binv.assign(TT, Eigen::MatrixXd::Zero(n, n));
    Binv[0].setIdentity();
    Binv[0] = Binv[0] * 1e10;
    r.assign(TT, Eigen::VectorXd::Zero(n));
    R.assign(TT, Eigen::MatrixXd::Zero(n, n));
    rhat = Eigen::VectorXd::Zero(TT);
    qhat.assign(TT, Eigen::VectorXd::Zero(n));
    linSolverTmp.resize(n, n);
    {
      if (dynamic)
      {
        q.resize(TT);
        for (int i = 0; i < q.size(); i++)
          q.at(i) = b.at(i).head(n2);
        if (prob_->getW().rows() != n2)
        {
          INDICATE_FAILURE
          ;
          ERROR(prob_->getW().rows()<<"!="<<n2);
          return PAR_ERR;
        }
      }
      else
      {
        q = b;
        if (prob_->getW().rows() != n)
        {
          INDICATE_FAILURE
          ;
          ERROR(prob_->getW().rows()<<"!="<<n);
          return PAR_ERR;
        }
      }
      // All the process parameters are assumed to be constant throughout the trajectory.
      // This is possible for a pseudo-dynamic system.
      // A dynamic system would have to update these based on forces acting on the system.
      Eigen::MatrixXd A_(n, n), B_(dynamic ? n2 : n, dynamic ? n2 : n), tB_,
          tA_, Ainv_, invtA_;
      Eigen::VectorXd a_(n);
      getProcess(A_, a_, B_);
      tB_ = B_.transpose();
      tA_ = A_.transpose();
      Ainv_ = A_.inverse();
      invtA_ = Ainv_.transpose();
      B.assign(TT, B_);
      tB.assign(TT, tB_);
      A.assign(TT, A_);
      tA.assign(TT, tA_);
      Ainv.assign(TT, Ainv_);
      invtA.assign(TT, invtA_);
      a.assign(TT, a_);
    }
    {
      // Set constant W,Win,H,Hinv
      Eigen::MatrixXd tmp;
      tmp = prob_->getW() * prob_->getWrate();
      W.assign(TT, tmp);
      tmp = (prob_->getW() * prob_->getWrate()).inverse();
      Winv.assign(TT, tmp);
      tmp = prob_->getW() * prob_->getHrate();
      H.assign(TT, tmp);
      tmp = (prob_->getW() * prob_->getHrate()).inverse();
      Hinv.assign(TT, tmp);
      tmp.setZero();
      tmp.diagonal().setConstant(prob_->getQrate());
      Q.assign(TT, tmp);
    }
    int m = 0;
    //for (TaskDefinition_map::const_iterator it=prob_->getTaskDefinitions().begin(); it!=prob_->getTaskDefinitions().end(); ++it)

    m = dim.sum();
    if (m == 0)
    {
      ERROR("No tasks were found!");
      return FAILURE;
    }
    phiBar.assign(TT, Eigen::VectorXd::Zero(m));
    JBar.assign(TT, Eigen::MatrixXd::Zero(m, dynamic ? n2 : n));
    y_star.assign(TT, Eigen::VectorXd::Zero(m));
    rhos.assign(TT, Eigen::VectorXd::Zero(prob_->getTaskDefinitions().size()));

    costControl.resize(T + 1);
    costControl.setZero();
    costTask.resize(T + 1, taskNames.size());
    costTask.setZero();
    {
      int cnt = 0, i = 0;
      for (auto & it : prob_->getTaskDefinitions())
      {
        boost::shared_ptr<TaskSqrError> task = boost::static_pointer_cast<
            TaskSqrError>(it.second);
        for (int t = 0; t < TT; t++)
        {
          task->registerGoal(
              Eigen::VectorXdRef_ptr(y_star[t].block(cnt, 0, dim(i), 1)), t);
          task->registerJacobian(
              Eigen::MatrixXdRef_ptr(
                  JBar[t].block(cnt, 0, dim(i), dynamic ? n2 : n)), t);
          task->registerPhi(
              Eigen::VectorXdRef_ptr(phiBar[t].block(cnt, 0, dim(i), 1)), t);
          task->registerRho(Eigen::VectorXdRef_ptr(rhos[t].block(i, 0, 1, 1)),
              t);
          task->setDefaultGoals(t);
        }
        cnt += dim(i);
        i++;
      }
    }

    q_stat.resize(T + 1);
    for (int t = 0; t <= T; t++)
    {
      q_stat[t].resize(n);
    }
    return SUCCESS;
  }

  void AICOsolver::getProcess(Eigen::Ref<Eigen::MatrixXd> A_,
      Eigen::Ref<Eigen::VectorXd> a_, Eigen::Ref<Eigen::MatrixXd> B_)
  {
    if (!dynamic)
    {
      A_ = Eigen::MatrixXd::Identity(n, n);
      B_ = Eigen::MatrixXd::Identity(n, n);
      a_ = Eigen::VectorXd::Zero(n);
    }
    else
    {
      double tau = prob_->getTau();
      int n2 = n / 2;
      A_ = Eigen::MatrixXd::Identity(n, n);
      B_ = Eigen::MatrixXd::Zero(n, n2);
      a_ = Eigen::VectorXd::Zero(n);

      // Assuming pseudo dynamic process (M is identity matrix and F is zero vector)
      Eigen::MatrixXd Minv = Eigen::MatrixXd::Identity(n2, n2);
      Eigen::VectorXd F = Eigen::VectorXd::Zero(n2);

      A_.topRightCorner(n2, n2).diagonal().setConstant(tau);
      B_.topLeftCorner(n2, n2) = Minv * (tau * tau * 0.5);
      B_.bottomLeftCorner(n2, n2) = Minv * tau;
      a_.head(n2) = Minv * F * (tau * tau * 0.5);
      a_.tail(n2) = Minv * F * tau;
    }
  }

  EReturn AICOsolver::initTrajectory(const std::vector<Eigen::VectorXd>& q_init)
  {
    if (q_init.size() != TT)
    {
      INDICATE_FAILURE
      ;
      return PAR_ERR;
    }
    qhat[0] = q_init[0];
    dampingReference[0] = q_init[0];
    b[0] = q_init[0];
    s[0] = q_init[0];
    rememberOldState();
    int n2 = n / 2;
    b = q_init;
    for (int i = 0; i < q.size(); i++)
      q.at(i) = b.at(i).head(n2);
    s = b;
    for (int t = 1; t <= T; t++)
      Sinv.at(t).diagonal().setConstant(damping);
    v = b;
    for (int t = 0; t <= T; t++)
      Vinv.at(t).diagonal().setConstant(damping);
    dampingReference = b;
    for (int t = 0; t <= T; t++)
    {
      // Compute task message reference
      if (!updateTaskMessage(t, b.at(t), 1.0) || !ros::ok())
      {
        return FAILURE;
      }
    }
    cost = evaluateTrajectory(b, true);
    if (cost < 0) return FAILURE;
    INFO(
        "Initial cost(ctrl/task/total): " << costControl.sum() << "/" << costTask.sum() << "/" << cost <<", updates: "<<updateCount);
    rememberOldState();
    return SUCCESS;
  }

  EReturn AICOsolver::inverseSymPosDef(Eigen::Ref<Eigen::MatrixXd> Ainv_,
      const Eigen::Ref<const Eigen::MatrixXd> & A_)
  {
    Ainv_ = A_;
    double* AA = Ainv_.data();
    integer info;
    integer nn = A_.rows();
    // Compute Cholesky
    dpotrf_((char*) "L", &nn, AA, &nn, &info);
    if (info != 0)
    {
      ERROR(info<<"\n"<<A_);
      return FAILURE;
    }
    // Invert
    dpotri_((char*) "L", &nn, AA, &nn, &info);
    if (info != 0)
    {
      ERROR(info);
      return FAILURE;
    }
    Ainv_.triangularView<Eigen::Upper>() = Ainv_.transpose();
    return SUCCESS;
  }

  EReturn AICOsolver::AinvBSymPosDef(Eigen::Ref<Eigen::VectorXd> x_,
      const Eigen::Ref<const Eigen::MatrixXd> & A_,
      const Eigen::Ref<const Eigen::VectorXd> & b_)
  {
    integer n_ = n, m_ = 1;
    integer info;
    linSolverTmp = A_;
    x_ = b_;
    double* AA = linSolverTmp.data();
    double* xx = x_.data();
    dposv_((char*) "L", &n_, &m_, AA, &n_, xx, &n_, &info);
    if (info != 0)
    {
      ERROR(info);
      return FAILURE;
    }
    return SUCCESS;
  }

  void AICOsolver::updateFwdMessage(int t)
  {
    Eigen::MatrixXd barS(n, n), St;
    if (dynamic)
    {
      MAT_OK(inverseSymPosDef(barS, Sinv[t - 1] + R[t - 1]));
      St = Q[t - 1] + B[t - 1] * Hinv[t - 1] * tB[t - 1]
          + A[t - 1] * barS * tA[t - 1];
      s[t] = a[t - 1] + A[t - 1] * (barS * (Sinv[t - 1] * s[t - 1] + r[t - 1]));
      MAT_OK(inverseSymPosDef(Sinv[t], St));
    }
    else
    {
      MAT_OK(inverseSymPosDef(barS, Sinv[t - 1] + R[t - 1]));
      s[t] = barS * (Sinv[t - 1] * s[t - 1] + r[t - 1]);
      St = Winv[t - 1] + barS;
      MAT_OK(inverseSymPosDef(Sinv[t], St));
    }
  }

  void AICOsolver::updateBwdMessage(int t)
  {
    Eigen::MatrixXd barV(n, n), Vt;
    if (dynamic)
    {
      if (t < T)
      {
        MAT_OK(inverseSymPosDef(barV, Vinv[t + 1] + R[t + 1]));
        Vt = Ainv[t] * (Q[t] + B[t] * Hinv[t] * tB[t] + barV) * invtA[t];
        v[t] = Ainv[t] * (-a[t] + barV * (Vinv[t + 1] * v[t + 1] + r[t + 1]));
        MAT_OK(inverseSymPosDef(Vinv[t], Vt));
      }
      if (t == T)
      {
        if (!useBwdMsg)
        {
          v[t] = b[t];
          Vinv[t].diagonal().setConstant(1e-4);
        }
        else
        {
          v[T] = bwdMsg_v;
          Vinv[T] = bwdMsg_Vinv;
        }
      }
    }
    else
    {
      if (t < T)
      {
        MAT_OK(inverseSymPosDef(barV, Vinv[t + 1] + R[t + 1]));
        v[t] = barV * (Vinv[t + 1] * v[t + 1] + r[t + 1]);
        Vt = Winv[t] + barV;
        MAT_OK(inverseSymPosDef(Vinv[t], Vt));
      }
      if (t == T)
      {
        if (!useBwdMsg)
        {
          v[t] = b[t];
          Vinv[t].diagonal().setConstant(1e-0);
        }
        else
        {
          v[T] = bwdMsg_v;
          Vinv[T] = bwdMsg_Vinv;
        }
      }
    }
  }

  bool AICOsolver::updateTaskMessage(int t,
      const Eigen::Ref<const Eigen::VectorXd> & qhat_t, double tolerance_,
      double maxStepSize)
  {
    Eigen::VectorXd diff = qhat_t - qhat[t];
    if ((diff.array().abs().maxCoeff() < tolerance_)) return true;
    double nrm = diff.norm();
    if (maxStepSize > 0. && nrm > maxStepSize)
    {
      qhat[t] += diff * (maxStepSize / nrm);
    }
    else
    {
      qhat[t] = qhat_t;
    }

    if (ok(prob_->update(dynamic ? qhat[t].head(n / 2) : qhat[t], t)))
    {
      updateCount++;
      double c = getTaskCosts(t);
      q_stat[t].addw(c > 0 ? 1.0 / (1.0 + c) : 1.0, qhat_t);
      return true;
    }
    else
    {
      INDICATE_FAILURE
      ;
      return false;
    }
    // If using fully dynamic system, update Q, Hinv and process variables here.
  }

  double AICOsolver::getTaskCosts(int t)
  {
    double C = 0;
    int n2 = n / 2;
    if (!dynamic)
    {
      Eigen::MatrixXd Jt;
      double prec;
      rhat[t] = 0;
      R[t].setZero();
      r[t].setZero();
      int offset = 0;
      for (int i = 0; i < prob_->getTaskDefinitions().size(); i++)
      {
        prec = rhos[t](i);
        if (prec > 0)
        {
          Jt = JBar[t].middleRows(offset, dim(i)).transpose();
          C += prec
              * (y_star[t].segment(offset, dim(i))
                  - phiBar[t].segment(offset, dim(i))).squaredNorm();
          R[t] += prec * Jt * JBar[t].middleRows(offset, dim(i));
          r[t] += prec * Jt
              * (y_star[t].segment(offset, dim(i))
                  - phiBar[t].segment(offset, dim(i))
                  + JBar[t].middleRows(offset, dim(i)) * qhat[t]);
          rhat[t] += prec
              * (y_star[t].segment(offset, dim(i))
                  - phiBar[t].segment(offset, dim(i))
                  + JBar[t].middleRows(offset, dim(i)) * qhat[t]).squaredNorm();
        }
        offset += dim(i);
      }
    }
    else
    {
      Eigen::VectorXd v;
      Eigen::MatrixXd Jt;
      double prec;
      double tau = prob_->getTau();
      rhat[t] = 0;
      R[t].setZero();
      r[t].setZero();
      int offset = 0, i = 0;
      for (auto& task_ : prob_->getTaskDefinitions())
      {
        if (task_.second->order == 0)
        {
          prec = rhos[t](i);
          if (prec > 0)
          {
            Jt = JBar[t].middleRows(offset, dim(i)).transpose();
            C += prec
                * (y_star[t].segment(offset, dim(i))
                    - phiBar[t].segment(offset, dim(i))).squaredNorm();
            R[t].topLeftCorner(n2, n2) += prec * Jt
                * JBar[t].middleRows(offset, dim(i));
            r[t].head(n2) += prec * Jt
                * (y_star[t].segment(offset, dim(i))
                    - phiBar[t].segment(offset, dim(i))
                    + JBar[t].middleRows(offset, dim(i)) * qhat[t]);
            rhat[t] +=
                prec
                    * (y_star[t].segment(offset, dim(i))
                        - phiBar[t].segment(offset, dim(i))
                        + JBar[t].middleRows(offset, dim(i)) * qhat[t]).squaredNorm();
          }
        }
        else if (task_.second->order == 1)
        {
          prec = rhos[t](i);
          if (prec > 0)
          {
            v = (phiBar[t].segment(offset, dim(i))
                - phiBar[t > 0 ? t - 1 : T + 1].segment(offset, dim(i))) / tau; // (phi_t-phi_{t-1})/tau
            Jt = JBar[t].middleRows(offset, dim(i)).transpose();
            C +=
                prec
                    * (v
                        - JBar[t].middleRows(offset, dim(i))
                            * (qhat[t].head(n / 2)
                                - qhat[t > 0 ? t - 1 : T + 1].head(n / 2)) / tau).squaredNorm(); // prec*J*q_dot; qdot=(qhat_t-q_hat_{t-1})/tau
            R[t].bottomRightCorner(n2, n2) += prec * Jt
                * JBar[t].middleRows(offset, dim(i));
            r[t].tail(n2) += prec * Jt * v;
            rhat[t] += prec * (v).squaredNorm();
          }
        }
        else
        {
          ERROR("Task variable " << task_.first << " is not supported!");
          return FAILURE;
        }
        offset += dim(i);
        i++;
      }
    }
    return C;
  }

  bool AICOsolver::updateTimeStep(int t, bool updateFwd, bool updateBwd,
      int maxRelocationIterations, double tolerance_, bool forceRelocation,
      double maxStepSize)
  {
    if (updateFwd) updateFwdMessage(t);
    if (updateBwd) updateBwdMessage(t);
    Eigen::VectorXd tmp;

    if (damping)
    {
      Binv[t] = Sinv[t] + Vinv[t] + R[t]
          + Eigen::MatrixXd::Identity(n, n) * damping;
      MAT_OK(
          AinvBSymPosDef(b[t], Binv[t],
              Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]
                  + damping * dampingReference[t]));
    }
    else
    {
      Binv[t] = Sinv[t] + Vinv[t] + R[t];
      MAT_OK(
          AinvBSymPosDef(b[t], Binv[t], Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]));
    }

    for (int k = 0; k < maxRelocationIterations; k++)
    {
      if (!((!k && forceRelocation)
          || (b[t] - qhat[t]).array().abs().maxCoeff() > tolerance_)) break;

      if (updateTaskMessage(t, b[t], 0., maxStepSize) && ros::ok())
      {

        //optional reUpdate fwd or bwd message (if the Dynamics might have changed...)
        if (updateFwd) updateFwdMessage(t);
        if (updateBwd) updateBwdMessage(t);

        if (damping)
        {
          Binv[t] = Sinv[t] + Vinv[t] + R[t]
              + Eigen::MatrixXd::Identity(n, n) * damping;
          MAT_OK(
              AinvBSymPosDef(b[t], Binv[t],
                  Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]
                      + damping * dampingReference[t]));
        }
        else
        {
          Binv[t] = Sinv[t] + Vinv[t] + R[t];
          MAT_OK(
              AinvBSymPosDef(b[t], Binv[t],
                  Sinv[t] * s[t] + Vinv[t] * v[t] + r[t]));
        }
      }
      else
      {
        INDICATE_FAILURE
        ;
        return false;
      }
    }
    return true;
  }

  bool AICOsolver::updateTimeStepGaussNewton(int t, bool updateFwd,
      bool updateBwd, int maxRelocationIterations, double tolerance,
      double maxStepSize)
  {
    // TODO: implement updateTimeStepGaussNewton
    return false;
  }

  double AICOsolver::evaluateTrajectory(const std::vector<Eigen::VectorXd>& x,
      bool skipUpdate)
  {
    ROS_WARN_STREAM("Evaluating, sweep "<<sweep);
    ros::Time start = ros::Time::now(), tmpTime;
    ros::Duration dSet, dPre, dUpd, dCtrl, dTask;
    double ret = 0.0;
    double tau = prob_->getTau();
    double tau_1 = 1. / tau, tau_2 = tau_1 * tau_1;
    int n2 = n / 2;
    Eigen::VectorXd vv;

    if (dynamic)
    {
      for (int i = 0; i < q.size(); i++)
        q.at(i) = x.at(i).head(n / 2);
    }
    else
    {
      q = x;
    }
    dSet = ros::Time::now() - start;
    if (preupdateTrajectory_)
    {
      ROS_WARN_STREAM("Pre-update, sweep "<<sweep);
      for (int t = 0; t <= T; t++)
      {
        if (!ros::ok()) return -1.0;
        updateCount++;
        if (!ok(prob_->update(q[t], t)))
        {
          INDICATE_FAILURE
          ;
          return -1;
        }
      }
    }
    dPre = ros::Time::now() - start - dSet;

    for (int t = 0; t <= T; t++)
    {
      tmpTime = ros::Time::now();
      if (!ros::ok()) return -1.0;
      if (!skipUpdate)
      {
        updateCount++;
        if (!ok(prob_->update(q[t], t)))
        {
          INDICATE_FAILURE
          ;
          return -1;
        }
      }
      dUpd += ros::Time::now() - tmpTime;
      tmpTime = ros::Time::now();

      // Control cost
      if (!dynamic)
      {
        if (t == 0)
        {
          costControl(t) = 0.0;
        }
        else
        {
          vv = q[t] - q[t - 1];
          costControl(t) = vv.transpose() * W[t] * vv;
        }
      }
      else
      {
        if (t < T && t > 0)
        {
          // For fully dynamic system use: v=tau_2*M*(q[t+1]+q[t-1]-2.0*q[t])-F;
          vv = tau_2 * (q[t + 1] + q[t - 1] - 2.0 * q[t]);
          costControl(t) = vv.transpose() * H[t] * vv;
        }
        else if (t == 0)
        {
          // For fully dynamic system use: v=tau_2*M*(q[t+1]+q[t])-F;
          vv = tau_2 * (q[t + 1] + q[t]);
          costControl(t) = vv.transpose() * H[t] * vv;
        }
        else
          costControl(t) = 0.0;
      }

      ret += costControl(t);
      dCtrl += ros::Time::now() - tmpTime;
      tmpTime = ros::Time::now();
      // Task cost
      double prec;
      int i = 0, offset = 0;
      for (auto& task_ : prob_->getTaskDefinitions())
      {
        if (task_.second->order == 0)
        {
          // Position cost
          prec = rhos[t](i);
          if (prec > 0)
          {
            costTask(t, i) = prec
                * (y_star[t].segment(offset, dim(i))
                    - phiBar[t].segment(offset, dim(i))).squaredNorm();
            ret += costTask(t, i);
          }
        }
        else if (dynamic && task_.second->order == 1)
        {
          // Velocity cost
          prec = rhos[t](i);
          if (prec > 0)
          {
            vv = (phiBar[t].segment(offset, dim(i))
                - phiBar[t > 0 ? t - 1 : T + 1].segment(offset, dim(i))) / tau; // (phi_t-phi_{t-1})/tau
            costTask(t, i) =
                prec
                    * (vv
                        - JBar[t].middleRows(offset, dim(i))
                            * (qhat[t].head(n / 2)
                                - qhat[t > 0 ? t - 1 : T + 1].head(n / 2)) / tau).squaredNorm(); // prec*J*q_dot; qdot=(qhat_t-q_hat_{t-1})/tau
            ret += costTask(t, i);
          }
        }
        else
        {
          ERROR("Task variable " << task_.first << " is not supported!");
          return -1;
        }
        offset += dim(i);
        i++;
      }
      dTask += ros::Time::now() - tmpTime;
    }
    //ROS_WARN_STREAM("Evaluation timing:\nState set: "<<dSet.toSec()<<"s\nPreupdate: "<< dPre.toSec()<<"s\nUpdate: "<< dUpd.toSec()<<"s\nControl: "<< dCtrl.toSec()<<"s\nTask: "<< dTask.toSec()<<"s\nTotal: "<<(dSet+dPre+dUpd+dCtrl+dTask).toSec()<<"s");
    return ret;
  }

  double AICOsolver::step()
  {
    rememberOldState();
    int t;
    switch (sweepMode)
    {
    //NOTE: the dependence on (Sweep?..:..) could perhaps be replaced by (DampingReference.N?..:..)
    case smForwardly:
      for (t = 1; t <= T; t++)
      {
        if (!updateTimeStep(t, true, false, 1, tolerance, !sweep, 1.)) //relocate once on fwd Sweep
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      for (t = T + 1; --t;)
      {
        if (!updateTimeStep(t, false, true, 0, tolerance, false, 1.)) //...not on bwd Sweep
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      break;
    case smSymmetric:
      ROS_WARN_STREAM("Updating forward, sweep "<<sweep);
      for (t = 1; t <= T; t++)
      {
        if (!updateTimeStep(t, true, false, 1, tolerance, !sweep, 1.)) //relocate once on fwd & bwd Sweep
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      ROS_WARN_STREAM("Updating backward, sweep "<<sweep);
      for (t = T + 1; --t;)
      {
        if (!updateTimeStep(t, false, true, (sweep ? 1 : 0), tolerance, false,
            1.))
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      break;
    case smLocalGaussNewton:
      for (t = 1; t <= T; t++)
      {
        if (!updateTimeStep(t, true, false, (sweep ? 5 : 1), tolerance, !sweep,
            1.)) //relocate iteratively on
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      for (t = T + 1; --t;)
      {
        if (!updateTimeStep(t, false, true, (sweep ? 5 : 0), tolerance, false,
            1.)) //...fwd & bwd Sweep
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      break;
    case smLocalGaussNewtonDamped:
      for (t = 1; t <= T; t++)
      {
        if (!updateTimeStepGaussNewton(t, true, false, (sweep ? 5 : 1),
            tolerance, 1.)) //GaussNewton in fwd & bwd Sweep
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      for (t = T + 1; --t;)
      {
        if (!updateTimeStep(t, false, true, (sweep ? 5 : 0), tolerance, false,
            1.))
        {
          INDICATE_FAILURE
          ;
          return -1.0;
        }
      }
      break;
    default:
      ERROR("non-existing Sweep mode")
      ;
      break;
    }
    b_step = 0.0;
    for (t = 0; t < b.size(); t++)
    {
      b_step = std::max((b_old[t] - b[t]).array().abs().maxCoeff(), b_step);
    }
    dampingReference = b;
    // q is set inside of evaluateTrajectory() function
    cost = evaluateTrajectory(b);
    if (cost < 0) return -1.0;
    INFO(
        "Sweep: " << sweep << ", updates: " << updateCount << ", cost(ctrl/task/total): " << costControl.sum() << "/" << costTask.sum() << "/" << cost << " (dq="<<b_step<<", damping="<<damping<<")");

    if (sweep && damping) perhapsUndoStep();
    sweep++;
    return b_step;
  }

  void AICOsolver::rememberOldState()
  {
    s_old = s;
    Sinv_old = Sinv;
    v_old = v;
    Vinv_old = Vinv;
    r_old = r;
    R_old = R;
    Binv_old = Binv;
    rhat_old = rhat;
    b_old = b;
    r_old = r;
    q_old = q;
    qhat_old = qhat;
    cost_old = cost;
    phiBar_old = phiBar;
    JBar_old = JBar;
    y_star_old = y_star;
    costControl_old = costControl;
    costTask_old = costTask;
    dim_old = dim;
    rhos_old = rhos;
  }

  void AICOsolver::perhapsUndoStep()
  {
    if (cost > cost_old)
    {
      damping *= 10.;
      s = s_old;
      Sinv = Sinv_old;
      v = v_old;
      Vinv = Vinv_old;
      r = r_old;
      R = R_old;
      Binv = Binv_old;
      rhat = rhat_old;
      b = b_old;
      r = r_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      phiBar = phiBar_old;
      JBar = JBar_old;
      y_star = y_star_old;
      dampingReference = b_old;
      costControl = costControl_old;
      costTask = costTask_old;
      dim = dim_old;
      rhos = rhos_old;
      if (preupdateTrajectory_)
      {
        for (int t = 0; t <= T; t++)
        {
          updateCount++;
          if (!ok(prob_->update(q[t], t)))
          {
            INDICATE_FAILURE
            ;
            return;
          }
        }
      }
      INFO("Reverting to previous step");
    }
    else
    {
      damping /= 5.;
    }
  }

} /* namespace exotica */
