/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#ifndef TIMEINDEXEDPROBLEM_H_
#define TIMEINDEXEDPROBLEM_H_

#include <exotica/PlanningProblem.h>
#include <exotica/Tasks.h>
#include <exotica/TimeIndexedProblemInitializer.h>

namespace exotica
{
/**
   * \brief Arbitrarily constrained time-indexed problem.
   */
class TimeIndexedProblem : public PlanningProblem, public Instantiable<TimeIndexedProblemInitializer>
{
public:
    TimeIndexedProblem();
    virtual ~TimeIndexedProblem();
    virtual void Instantiate(TimeIndexedProblemInitializer& init);
    double getDuration();
    void Update(Eigen::VectorXdRefConst x, int t);
    std::vector<Eigen::VectorXd> getInitialTrajectory();
    void setInitialTrajectory(const std::vector<Eigen::VectorXd> q_init_in);
    virtual void preupdate();
    void setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void setRho(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd getGoal(const std::string& task_name, int t = 0);
    double getRho(const std::string& task_name, int t = 0);
    void setGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void setRhoEQ(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd getGoalEQ(const std::string& task_name, int t = 0);
    double getRhoEQ(const std::string& task_name, int t = 0);
    void setGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal, int t = 0);
    void setRhoNEQ(const std::string& task_name, const double rho, int t = 0);
    Eigen::VectorXd getGoalNEQ(const std::string& task_name, int t = 0);
    double getRhoNEQ(const std::string& task_name, int t = 0);
    std::vector<double>& getBounds();

    int getT() const { return T; }
    void setT(int T_in);

    double getTau() const { return tau; }
    void setTau(double tau_in);

    double getScalarTaskCost(int t);
    Eigen::VectorXd getScalarTaskJacobian(int t);
    double getScalarTransitionCost(int t);
    Eigen::VectorXd getScalarTransitionJacobian(int t);

    double ct;  //!< Normalisation of scalar cost and Jacobian over trajectory length
    TimeIndexedTask Cost;
    TimeIndexedTask Inequality;
    TimeIndexedTask Equality;

    double W_rate;  //!< Kinematic system transition error covariance multiplier (constant throughout the trajectory)
    Eigen::MatrixXd W;

    std::vector<TaskSpaceVector> Phi;
    std::vector<Eigen::MatrixXd> J;

    std::vector<Eigen::VectorXd> x;      // current internal problem state
    std::vector<Eigen::VectorXd> xdiff;  // equivalent to dx = x(t)-x(t-1)

    int PhiN;
    int JN;
    int NumTasks;
    bool useBounds;

private:
    int T;       //!< Number of time steps
    double tau;  //!< Time step duration

    std::vector<Eigen::VectorXd> InitialTrajectory;
    std::vector<double> bounds_;
    TimeIndexedProblemInitializer init_;
    void reinitializeVariables();
};

typedef std::shared_ptr<exotica::TimeIndexedProblem> TimeIndexedProblem_ptr;
}

#endif
