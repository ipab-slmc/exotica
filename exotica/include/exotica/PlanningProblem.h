/*
 *      Author: Michael Camilleri
 *
 * Copyright (c) 2016, University of Edinburgh
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

#ifndef EXOTICA_MOTION_PLANNING_PROBLEM_H
#define EXOTICA_MOTION_PLANNING_PROBLEM_H

#include <exotica/Object.h>
#include <exotica/Scene.h>
#include <exotica/Server.h>
#include <exotica/TaskMap.h>
#include <exotica/TaskSpaceVector.h>
#include <exotica/Tools.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#define REGISTER_PROBLEM_TYPE(TYPE, DERIV) EXOTICA_REGISTER_CORE(exotica::PlanningProblem, TYPE, DERIV)

namespace exotica
{
enum class TerminationCriterion
{
    NotStarted = -1,
    // Continue = 0,
    IterationLimit,
    BacktrackIterationLimit,
    StepTolerance,
    FunctionTolerance,
    GradientTolerance,
    Divergence,
    UserDefined
    // Condition,
};

class PlanningProblem : public Object, Uncopyable, public virtual InstantiableBase, public std::enable_shared_from_this<PlanningProblem>
{
public:
    PlanningProblem();
    virtual ~PlanningProblem() {}
    virtual void InstantiateBase(const Initializer& init);
    TaskMap_map& getTaskMaps();
    TaskMap_vec& getTasks();
    Scene_ptr getScene();
    virtual std::string print(std::string prepend);
    void setStartState(Eigen::VectorXdRefConst x);
    void setStartTime(double t);
    Eigen::VectorXd getStartState();
    double getStartTime();
    Eigen::VectorXd applyStartState(bool updateTraj = true);
    int N;
    double tStart;
    TerminationCriterion terminationCriterion;
    virtual void preupdate();
    unsigned int getNumberOfProblemUpdates() { return numberOfProblemUpdates; }
    void resetNumberOfProblemUpdates() { numberOfProblemUpdates = 0; }
    std::pair<std::vector<double>, std::vector<double>> getCostEvolution();
    double getCostEvolution(int index);
    void resetCostEvolution(unsigned int size);
    void setCostEvolution(int index, double value);
    KinematicRequestFlags getFlags() { return Flags; }
    virtual bool isValid() { throw_named("Not implemented"); };
protected:
    void updateTaskKinematics(std::shared_ptr<KinematicResponse> response);
    void updateMultipleTaskKinematics(std::vector<std::shared_ptr<KinematicResponse>> responses);

    Scene_ptr scene_;
    TaskMap_map TaskMaps;
    TaskMap_vec Tasks;
    KinematicRequestFlags Flags;
    Eigen::VectorXd startState;
    unsigned int numberOfProblemUpdates = 0;  // Stores number of times the problem has been updated
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, double>> costEvolution;
};

typedef Factory<PlanningProblem> PlanningProblem_fac;
typedef std::shared_ptr<PlanningProblem> PlanningProblem_ptr;
}
#endif
