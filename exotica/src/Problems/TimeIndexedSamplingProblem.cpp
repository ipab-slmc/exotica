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

#include <exotica/Problems/TimeIndexedSamplingProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("TimeIndexedSamplingProblem", exotica::TimeIndexedSamplingProblem)

namespace exotica
{
TimeIndexedSamplingProblem::TimeIndexedSamplingProblem()
{
    Flags = KIN_FK;
}

TimeIndexedSamplingProblem::~TimeIndexedSamplingProblem()
{
    // TODO Auto-generated destructor stub
}

std::vector<double> TimeIndexedSamplingProblem::getBounds()
{
    std::vector<double> bounds;
    Eigen::MatrixXdRef jointLimits = scene_->getSolver().getJointLimits();

    bounds.resize(2 * N);
    for (unsigned int i = 0; i < N; i++)
    {
        bounds[i] = jointLimits(i, 0);
        bounds[i + N] = jointLimits(i, 1);
    }

    return bounds;
}

void TimeIndexedSamplingProblem::Instantiate(TimeIndexedSamplingProblemInitializer& init)
{
    Parameters = init;
    goal_ = init.Goal;
    if (goal_.rows() != N)
        throw_named("Dimension mismatch: problem N=" << N << ", but goal state has dimension " << goal_.rows());
    T = init.T;
    if (T < 0)
        throw_named("Invalid problem duration T: " << T);
    tGoal = init.GoalTime;
    if (tGoal > T)
        throw_named("Invalid goal time tGoal= " << tGoal << ">T_(" << T << ")");
    if (tGoal == -1.0)
        tGoal = T;
    vel_limits_ = init.JointVelocityLimits;
    if (vel_limits_.rows() != N)
        throw_named("Dimension mismatch: problem N=" << N << ", but joint velocity limits has dimension " << vel_limits_.rows());

    NumTasks = Tasks.size();
    PhiN = 0;
    JN = 0;
    for (int i = 0; i < NumTasks; i++)
    {
        appendVector(Phi.map, Tasks[i]->getLieGroupIndices());
        PhiN += Tasks[i]->Length;
        JN += Tasks[i]->LengthJ;
    }
    Phi.setZero(PhiN);

    Constraint.initialize(init.Constraint, shared_from_this(), ConstraintPhi);

    applyStartState(false);
    preupdate();
}

Eigen::VectorXd TimeIndexedSamplingProblem::getGoalState()
{
    return goal_;
}

double TimeIndexedSamplingProblem::getGoalTime()
{
    return tGoal;
}

void TimeIndexedSamplingProblem::setGoalState(Eigen::VectorXdRefConst qT)
{
    if (qT.rows() != N)
        throw_pretty("Dimensionality of goal state wrong: Got " << qT.rows() << ", expected " << N);
    goal_ = qT;
}

void TimeIndexedSamplingProblem::setGoalTime(double t)
{
    tGoal = t;
}

bool TimeIndexedSamplingProblem::isValid(Eigen::VectorXdRefConst x, double t)
{
    scene_->Update(x, t);
    for (int i = 0; i < NumTasks; i++)
    {
        if (Tasks[i]->isUsed)
            Tasks[i]->update(x, Phi.data.segment(Tasks[i]->Start, Tasks[i]->Length));
    }
    Constraint.update(Phi);
    numberOfProblemUpdates++;
    return ((Constraint.S * Constraint.ydiff).array() < 0.0).all();
}

void TimeIndexedSamplingProblem::preupdate()
{
    PlanningProblem::preupdate();
    for (int i = 0; i < Tasks.size(); i++) Tasks[i]->isUsed = false;
    Constraint.updateS();
}

void TimeIndexedSamplingProblem::Update(Eigen::VectorXdRefConst x, double t)
{
    isValid(x, t);
    numberOfProblemUpdates++;
}

int TimeIndexedSamplingProblem::getSpaceDim()
{
    return N;
}
} /* namespace exotica */
