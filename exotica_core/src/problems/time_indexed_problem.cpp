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

#include <exotica_core/problems/time_indexed_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("TimeIndexedProblem", exotica::TimeIndexedProblem)

namespace exotica
{
void TimeIndexedProblem::Instantiate(const TimeIndexedProblemInitializer& init)
{
    this->parameters_ = init;

    N = scene_->GetKinematicTree().GetNumControlledJoints();

    w_scale_ = this->parameters_.Wrate;
    W = Eigen::MatrixXd::Identity(N, N) * w_scale_;
    if (this->parameters_.W.rows() > 0)
    {
        if (this->parameters_.W.rows() == N)
        {
            W.diagonal() = this->parameters_.W * w_scale_;
        }
        else
        {
            ThrowNamed("W dimension mismatch! Expected " << N << ", got " << this->parameters_.W.rows());
        }
    }

    if (init.LowerBound.rows() == N)
    {
        scene_->GetKinematicTree().SetJointLimitsLower(init.LowerBound);
    }
    else if (init.LowerBound.rows() != 0)
    {
        ThrowNamed("Lower bound size incorrect! Expected " << N << " got " << init.LowerBound.rows());
    }
    if (init.UpperBound.rows() == N)
    {
        scene_->GetKinematicTree().SetJointLimitsUpper(init.UpperBound);
    }
    else if (init.UpperBound.rows() != 0)
    {
        ThrowNamed("Lower bound size incorrect! Expected " << N << " got " << init.UpperBound.rows());
    }

    use_bounds = this->parameters_.UseBounds;

    cost.Initialize(this->parameters_.Cost, shared_from_this(), cost_Phi);
    inequality.Initialize(this->parameters_.Inequality, shared_from_this(), inequality_Phi);
    equality.Initialize(this->parameters_.Equality, shared_from_this(), equality_Phi);

    T_ = this->parameters_.T;
    tau_ = this->parameters_.tau;
    SetJointVelocityLimits(this->parameters_.JointVelocityLimits);
    ApplyStartState(false);
    ReinitializeVariables();
}

bool TimeIndexedProblem::IsValid()
{
    bool succeeded = true;
    auto bounds = scene_->GetKinematicTree().GetJointLimits();

    std::cout.precision(4);

    // Check for every state
    for (int t = 0; t < T_; ++t)
    {
        // Check joint limits
        if (use_bounds)
        {
            for (int i = 0; i < N; ++i)
            {
                constexpr double tolerance = 1.e-3;
                if (x[t](i) < bounds(i, 0) - tolerance || x[t](i) > bounds(i, 1) + tolerance)
                {
                    if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "State at timestep " << t << " is out of bounds: joint #" << i << ": " << bounds(i, 0) << " < " << x[t](i) << " < " << bounds(i, 1));
                    succeeded = false;
                }
            }
        }

        // Check inequality constraints
        if (GetInequality(t).rows() > 0)
        {
            if (GetInequality(t).maxCoeff() > this->parameters_.InequalityFeasibilityTolerance)
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "Violated inequality constraints at timestep " << t << ": " << GetInequality(t).transpose());
                succeeded = false;
            }
        }

        // Check equality constraints
        if (GetEquality(t).rows() > 0)
        {
            if (GetEquality(t).cwiseAbs().maxCoeff() > this->parameters_.EqualityFeasibilityTolerance)
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "Violated equality constraints at timestep " << t << ": " << GetEquality(t).cwiseAbs().maxCoeff());
                succeeded = false;
            }
        }

        // Check joint velocity limits
        if (q_dot_max_.maxCoeff() > 0 && t > 0)
        {
            if (((x[t] - x[t - 1]).cwiseAbs() - xdiff_max_).maxCoeff() > 1.e-5)  // The 1e-5 are a required tolerance...
            {
                if (debug_) HIGHLIGHT_NAMED("TimeIndexedProblem::IsValid", "Violated joint velocity constraints at timestep " << t << ": (" << (x[t] - x[t - 1]).transpose() << "), (limit=" << xdiff_max_.transpose() << "), violation: (" << ((x[t] - x[t - 1]).cwiseAbs() - xdiff_max_).transpose() << ")");
                succeeded = false;
            }
        }
    }

    return succeeded;
}
}  // namespace exotica
