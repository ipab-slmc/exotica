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

#ifndef EXOTICA_MOTION_SOLVER_H
#define EXOTICA_MOTION_SOLVER_H

#include "exotica/Object.h"
#include "exotica/PlanningProblem.h"
#include "exotica/Property.h"
#include "exotica/Server.h"
#include "exotica/TaskMap.h"

#define REGISTER_MOTIONSOLVER_TYPE(TYPE, DERIV) EXOTICA_REGISTER(exotica::MotionSolver, TYPE, DERIV)

namespace exotica
{
class MotionSolver : public Object, Uncopyable, public virtual InstantiableBase
{
public:
    MotionSolver();
    virtual ~MotionSolver() {}
    virtual void InstantiateBase(const Initializer& init);
    virtual void specifyProblem(PlanningProblem_ptr pointer);
    virtual void Solve(Eigen::MatrixXd& solution) = 0;
    PlanningProblem_ptr getProblem() { return problem_; }
    virtual std::string print(std::string prepend);
    void setNumberOfMaxIterations(int maxIter)
    {
        if (maxIter < 1) throw_pretty("Number of maximum iterations needs to be greater than 0.");
        maxIterations_ = maxIter;
    }
    int getNumberOfMaxIterations() { return maxIterations_; }
    double getPlanningTime() { return planning_time_; }
protected:
    PlanningProblem_ptr problem_;
    double planning_time_ = -1;
    int maxIterations_ = 100;
};

typedef exotica::Factory<exotica::MotionSolver> MotionSolver_fac;
typedef std::shared_ptr<exotica::MotionSolver> MotionSolver_ptr;
}

#endif
