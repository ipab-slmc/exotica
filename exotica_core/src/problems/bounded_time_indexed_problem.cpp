//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <exotica_core/problems/bounded_time_indexed_problem.h>
#include <exotica_core/setup.h>

REGISTER_PROBLEM_TYPE("BoundedTimeIndexedProblem", exotica::BoundedTimeIndexedProblem)

namespace exotica
{
void BoundedTimeIndexedProblem::Instantiate(const BoundedTimeIndexedProblemInitializer& init)
{
    this->parameters_ = init;

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

    cost.Initialize(this->parameters_.Cost, shared_from_this(), cost_Phi);

    T_ = this->parameters_.T;
    tau_ = this->parameters_.tau;
    ApplyStartState(false);
    ReinitializeVariables();
}

void BoundedTimeIndexedProblem::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;
    cost.UpdateS();

    // Create a new set of kinematic solutions with the size of the trajectory
    // based on the lastest KinematicResponse in order to reflect model state
    // updates etc.
    kinematic_solutions_.clear();
    kinematic_solutions_.resize(T_);
    for (int i = 0; i < T_; ++i) kinematic_solutions_[i] = std::make_shared<KinematicResponse>(*scene_->GetKinematicTree().GetKinematicResponse());
}

void BoundedTimeIndexedProblem::Update(Eigen::VectorXdRefConst x_in, int t)
{
    ValidateTimeIndex(t);

    x[t] = x_in;

    // Set the corresponding KinematicResponse for KinematicTree in order to
    // have Kinematics elements updated based in x_in.
    scene_->GetKinematicTree().SetKinematicResponse(kinematic_solutions_[t]);

    // Pass the corresponding number of relevant task kinematics to the TaskMaps
    // via the PlanningProblem::UpdateMultipleTaskKinematics method. For now we
    // support passing _two_ timesteps - this can be easily changed later on.
    std::vector<std::shared_ptr<KinematicResponse>> kinematics_solutions{kinematic_solutions_[t]};

    // If the current timestep is 0, pass the 0th timestep's response twice.
    // Otherwise pass the (t-1)th response.
    kinematics_solutions.emplace_back((t == 0) ? kinematic_solutions_[t] : kinematic_solutions_[t - 1]);

    // Actually update the tasks' kinematics mappings.
    PlanningProblem::UpdateMultipleTaskKinematics(kinematics_solutions);

    scene_->Update(x_in, static_cast<double>(t) * tau_);

    Phi[t].SetZero(length_Phi);
    if (flags_ & KIN_J) jacobian[t].setZero();
    if (flags_ & KIN_H)
        for (int i = 0; i < length_jacobian; ++i) hessian[t](i).setZero();
    for (int i = 0; i < num_tasks; ++i)
    {
        // Only update TaskMap if rho is not 0
        if (tasks_[i]->is_used)
        {
            if (flags_ & KIN_H)
            {
                tasks_[i]->Update(x[t],
                                  Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length),
                                  jacobian[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian),
                                  hessian[t].segment(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian));
            }
            else if (flags_ & KIN_J)
            {
                tasks_[i]->Update(x[t], Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length), jacobian[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian));
            }
            else
            {
                tasks_[i]->Update(x[t], Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length));
            }
        }
    }
    if (flags_ & KIN_H)
    {
        cost.Update(Phi[t], jacobian[t], hessian[t], t);
    }
    else if (flags_ & KIN_J)
    {
        cost.Update(Phi[t], jacobian[t], t);
    }
    else
    {
        cost.Update(Phi[t], t);
    }

    if (t > 0) xdiff[t] = x[t] - x[t - 1];

    ++number_of_problem_updates_;
}

void BoundedTimeIndexedProblem::ReinitializeVariables()
{
    if (debug_) HIGHLIGHT_NAMED("BoundedTimeIndexedProblem", "Initialize problem with T=" << T_);

    num_tasks = tasks_.size();
    length_Phi = 0;
    length_jacobian = 0;
    TaskSpaceVector y_ref_;
    for (int i = 0; i < num_tasks; ++i)
    {
        AppendVector(y_ref_.map, tasks_[i]->GetLieGroupIndices());
        length_Phi += tasks_[i]->length;
        length_jacobian += tasks_[i]->length_jacobian;
    }

    y_ref_.SetZero(length_Phi);
    Phi.assign(T_, y_ref_);

    x.assign(T_, Eigen::VectorXd::Zero(N));
    xdiff.assign(T_, Eigen::VectorXd::Zero(N));
    if (flags_ & KIN_J) jacobian.assign(T_, Eigen::MatrixXd(length_jacobian, N));
    if (flags_ & KIN_H)
    {
        Hessian Htmp;
        Htmp.setConstant(length_jacobian, Eigen::MatrixXd::Zero(N, N));
        hessian.assign(T_, Htmp);
    }

    // Set initial trajectory with current state
    initial_trajectory_.resize(T_, scene_->GetControlledState());

    cost.ReinitializeVariables(T_, shared_from_this(), cost_Phi);

    // Updates related to tau
    ct = 1.0 / tau_ / T_;
    xdiff_max_ = q_dot_max_ * tau_;

    PreUpdate();
}

bool BoundedTimeIndexedProblem::IsValid()
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
    }

    return succeeded;
}
}  // namespace exotica
