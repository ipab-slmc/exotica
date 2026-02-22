//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_MOTION_SOLVER_H_
#define EXOTICA_CORE_MOTION_SOLVER_H_

#include <exotica_core/factory.h>
#include <exotica_core/object.h>
#include <exotica_core/planning_problem.h>
#include <exotica_core/property.h>

#define REGISTER_MOTIONSOLVER_TYPE(TYPE, DERIV) EXOTICA_CORE_REGISTER(exotica::MotionSolver, TYPE, DERIV)

namespace exotica
{
class MotionSolver : public Object, Uncopyable, public virtual InstantiableBase
{
public:
    MotionSolver() = default;
    virtual ~MotionSolver() = default;
    void InstantiateBase(const Initializer& init) override;
    virtual void SpecifyProblem(PlanningProblemPtr pointer);
    virtual void Solve(Eigen::MatrixXd& solution) = 0;
    PlanningProblemPtr GetProblem() const { return problem_; }
    std::string Print(const std::string& prepend) const override;
    void SetNumberOfMaxIterations(int max_iter)
    {
        if (max_iter < 1) ThrowPretty("Number of maximum iterations needs to be greater than 0.");
        max_iterations_ = max_iter;
    }
    int GetNumberOfMaxIterations() { return max_iterations_; }
    double GetPlanningTime() { return planning_time_; }

protected:
    PlanningProblemPtr problem_;
    double planning_time_ = -1;
    int max_iterations_ = 100;
};

typedef std::shared_ptr<exotica::MotionSolver> MotionSolverPtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_MOTION_SOLVER_H_
