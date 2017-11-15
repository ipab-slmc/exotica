/*
 *  Created on: 7 Nov 2017
 *      Author: Yiming Yang
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

#ifndef INCLUDE_EXOTICA_PROBLEMS_TIMEINDEXEDSAMPLINGPROBLEM_H_
#define INCLUDE_EXOTICA_PROBLEMS_TIMEINDEXEDSAMPLINGPROBLEM_H_

#include <exotica/PlanningProblem.h>
#include <exotica/TimeIndexedSamplingProblemInitializer.h>

namespace exotica
{
class TimeIndexedSamplingProblem : public PlanningProblem, public Instantiable<TimeIndexedSamplingProblemInitializer>
{
public:
    TimeIndexedSamplingProblem();
    virtual ~TimeIndexedSamplingProblem();

    virtual void Instantiate(TimeIndexedSamplingProblemInitializer& init);
    virtual Eigen::VectorXd applyStartState();

    void Update(Eigen::VectorXdRefConst x, double t);
    bool isValid(Eigen::VectorXdRefConst x, double t);

    int getSpaceDim();

    void setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void setThreshold(const std::string& task_name, Eigen::VectorXdRefConst threshold);
    void setRho(const std::string& task_name, const double rho);
    Eigen::VectorXd getGoal(const std::string& task_name);
    Eigen::VectorXd getThreshold(const std::string& task_name);
    double getRho(const std::string& task_name);

    std::vector<double>& getBounds();

    TimeIndexedSamplingProblemInitializer Parameters;

    void setStartTime(double t);
    void setGoalState(Eigen::VectorXdRefConst qT);
    void setGoalTime(double t);

    Eigen::VectorXd goal_;
    double T_;
    double T_start_;
    double T_goal_;
    Eigen::VectorXd vel_limits_;
    Eigen::VectorXd Rho;
    TaskSpaceVector y;
    TaskSpaceVector Phi;
    Eigen::VectorXd threshold_;

    int PhiN;
    int JN;
    int NumTasks;
    Eigen::MatrixXd S;

private:
    std::vector<double> bounds_;
};

typedef std::shared_ptr<exotica::TimeIndexedSamplingProblem> TimeIndexedSamplingProblem_ptr;
}

#endif /* INCLUDE_EXOTICA_PROBLEMS_TIMEINDEXEDSAMPLINGPROBLEM_H_ */
