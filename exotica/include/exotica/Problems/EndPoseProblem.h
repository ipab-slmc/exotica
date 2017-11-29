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

#ifndef ENDPOSEPROBLEM_H_
#define ENDPOSEPROBLEM_H_
#include <exotica/PlanningProblem.h>
#include <exotica/EndPoseProblemInitializer.h>
#include <exotica/Tasks.h>

namespace exotica
{
/**
    * Arbitrarily constrained end-pose problem implementation
    */
class EndPoseProblem : public PlanningProblem, public Instantiable<EndPoseProblemInitializer>
{
public:
    EndPoseProblem();
    virtual ~EndPoseProblem();

    virtual void Instantiate(EndPoseProblemInitializer& init);
    void Update(Eigen::VectorXdRefConst x);

    void setGoal(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void setRho(const std::string& task_name, const double rho);
    Eigen::VectorXd getGoal(const std::string& task_name);
    double getRho(const std::string& task_name);
    void setGoalEQ(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void setRhoEQ(const std::string& task_name, const double rho);
    Eigen::VectorXd getGoalEQ(const std::string& task_name);
    double getRhoEQ(const std::string& task_name);
    void setGoalNEQ(const std::string& task_name, Eigen::VectorXdRefConst goal);
    void setRhoNEQ(const std::string& task_name, const double rho);
    Eigen::VectorXd getGoalNEQ(const std::string& task_name);
    double getRhoNEQ(const std::string& task_name);
    virtual void preupdate();
    std::vector<double>& getBounds();

    double getScalarCost();
    Eigen::VectorXd getScalarJacobian();

    EndPoseTask Cost;
    EndPoseTask Inequality;
    EndPoseTask Equality;

    Eigen::MatrixXd W;
    TaskSpaceVector Phi;
    Eigen::MatrixXd J;

    int PhiN;
    int JN;
    int NumTasks;
    bool useBounds;

protected:
    void initTaskTerms(const std::vector<exotica::Initializer>& inits);
    std::vector<double> bounds_;
};
typedef std::shared_ptr<exotica::EndPoseProblem> EndPoseProblem_ptr;
}

#endif
