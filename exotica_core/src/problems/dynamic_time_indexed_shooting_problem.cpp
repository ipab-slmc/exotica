//
// Copyright (c) 2019, Wolfgang Merkt
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

#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>
#include <exotica_core/setup.h>
#include <cmath>

REGISTER_PROBLEM_TYPE("DynamicTimeIndexedShootingProblem", exotica::DynamicTimeIndexedShootingProblem)

namespace exotica
{
DynamicTimeIndexedShootingProblem::DynamicTimeIndexedShootingProblem()
{
    this->flags_ = KIN_FK | KIN_J;
}
DynamicTimeIndexedShootingProblem::~DynamicTimeIndexedShootingProblem() = default;

void DynamicTimeIndexedShootingProblem::Instantiate(const DynamicTimeIndexedShootingProblemInitializer& init)
{
    this->parameters_ = init;

    if (!scene_->GetDynamicsSolver()) ThrowPretty("DynamicsSolver is not initialised!");

    const int NX = num_positions_ + num_velocities_,
              NU = num_controls_;
    Qf_ = Eigen::MatrixXd::Identity(NX, NX);
    if (this->parameters_.Qf.rows() > 0)
    {
        if (this->parameters_.Qf.rows() == NX)
        {
            Qf_.diagonal() = this->parameters_.Qf;
        }
        else
        {
            ThrowNamed("Qf dimension mismatch! Expected " << NX << ", got " << this->parameters_.Qf.rows());
        }
    }
    Qf_ *= this->parameters_.Qf_rate;

    R_ = this->parameters_.R_rate * Eigen::MatrixXd::Identity(num_controls_, num_controls_);
    if (this->parameters_.R.rows() > 0)
    {
        if (this->parameters_.R.rows() == num_controls_)
        {
            R_.diagonal() = this->parameters_.R;
        }
        else
        {
            ThrowNamed("R dimension mismatch! Expected " << num_controls_ << ", got " << this->parameters_.R.rows());
        }
    }

    // Set up stochastic terms
    //  see https://homes.cs.washington.edu/~todorov/papers/TodorovNeuralComp05.pdf
    //  eq. 3.1 and the text before (search for 'column') to see why this makes sense
    //
    // We specify a matrix of size NX x NU from which the C matrices
    //  are extracted
    //
    // E.g. NU = 2, NX = 4
    //
    // The matrix is
    //  a b
    //  c d
    //  e f
    //  g h
    //
    // From which the Ci matrices become
    //  C0 = a b   C1 = 0 0   C2 = 0 0   C3 = 0 0
    //       0 0        c d        0 0        0 0
    //       0 0        0 0        e f        0 0
    //       0 0        0 0        0 0        g h
    //
    // If you specify C_rate, then this is equivalent to:
    //
    // C = 0 0
    //     0 0
    //     c 0
    //     0 c
    //
    // The velocities then take the noise terms in. For an
    //  underactuated system these are somewhat ill-defined. E.g.
    //  if above NU = 1 and you specify c:
    //
    // C = 0
    //     0
    //     c
    //     0
    bool full_noise_set = false;
    Ci_.assign(NX, Eigen::MatrixXd::Zero(NX, NU));
    for (int i = 0; i < NU; ++i)
        Ci_[NX - NU + i](NX - NU + i, i) = parameters_.C_rate;

    if (this->parameters_.C.rows() > 0)
    {
        if (parameters_.C.rows() * parameters_.C.cols() == NX * NU)
        {
            Eigen::Map<Eigen::MatrixXd> C_map(parameters_.C.data(), NU, NX);

            for (int i = 0; i < NX; ++i)
                Ci_[i].row(i) = C_map.col(i).transpose();  // row over vs. col order
            full_noise_set = true;
        }
        else
        {
            ThrowNamed("C dimension mismatch! Expected " << NX << "x" << NU << ", got " << parameters_.C.rows() << "x" << parameters_.C.cols());
        }
    }

    CW_ = this->parameters_.CW_rate * Eigen::MatrixXd::Identity(NX, NX);
    if (parameters_.CW.rows() > 0)
    {
        if (parameters_.CW.rows() == NX)
        {
            CW_.diagonal() = parameters_.CW;
            full_noise_set = true;
        }
        else
        {
            ThrowNamed("CW dimension mismatch! Expected " << NX << ", got " << parameters_.R.rows());
        }
    }

    if (parameters_.C_rate > 0 || parameters_.CW_rate > 0 || full_noise_set)
    {
        stochastic_matrices_specified_ = true;
        stochastic_updates_enabled_ = true;
    }

    T_ = this->parameters_.T;
    tau_ = this->parameters_.tau;

    // For now, without inter-/extra-polation for integrators, assure that tau is a multiple of dt
    const long double fmod_tau_dt = std::fmod(static_cast<long double>(1000. * tau_), static_cast<long double>(1000. * scene_->GetDynamicsSolver()->get_dt()));
    if (fmod_tau_dt > 1e-5) ThrowPretty("tau is not a multiple of dt: tau=" << tau_ << ", dt=" << scene_->GetDynamicsSolver()->get_dt() << ", mod(" << fmod_tau_dt << ")");

    // Initialize general costs
    cost.Initialize(this->parameters_.Cost, shared_from_this(), cost_Phi);

    ApplyStartState(false);
    ReinitializeVariables();
}

void DynamicTimeIndexedShootingProblem::ReinitializeVariables()
{
    if (debug_) HIGHLIGHT_NAMED("DynamicTimeIndexedShootingProblem", "Initialize problem with T=" << T_);

    const int NX = num_positions_ + num_velocities_;
    X_ = Eigen::MatrixXd::Zero(NX, T_);
    X_star_ = Eigen::MatrixXd::Zero(NX, T_);
    U_ = Eigen::MatrixXd::Zero(num_controls_, T_ - 1);

    // Set GoalState
    if (this->parameters_.GoalState.rows() > 0)
    {
        Eigen::MatrixXd goal_state = Eigen::MatrixXd::Zero(NX, T_);
        if (this->parameters_.GoalState.rows() == NX)
        {
            goal_state.col(T_ - 1) = this->parameters_.GoalState;
        }
        else if (this->parameters_.GoalState.rows() == num_positions_)
        {
            goal_state.col(T_ - 1).head(num_positions_) = this->parameters_.GoalState;
        }
        else if (this->parameters_.GoalState.rows() == NX * T_)
        {
            for (int t = 0; t < T_; ++t)
            {
                goal_state.col(t) = this->parameters_.GoalState.segment(t * NX, NX);
            }
        }
        else
        {
            ThrowPretty("GoalState has " << this->parameters_.GoalState.rows() << " rows, but expected either NX=" << NX << " or NQ=" << num_positions_ << ", or NX*T=" << NX * T_);
        }
        set_X_star(goal_state);
    }

    // Set StartState
    if (this->parameters_.StartState.rows() > 0)
    {
        Eigen::MatrixXd start_state = Eigen::MatrixXd::Zero(NX, T_);
        if (this->parameters_.StartState.rows() == NX)
        {
            start_state = this->parameters_.StartState.replicate(1, T_);
        }
        else if (this->parameters_.StartState.rows() == num_positions_)
        {
            for (int t = 0; t < T_; ++t)
            {
                start_state.col(t).head(num_positions_) = this->parameters_.StartState;
            }
        }
        else if (this->parameters_.StartState.rows() == NX * T_)
        {
            for (int t = 0; t < T_; ++t)
            {
                start_state.col(t) = this->parameters_.StartState.segment(t * NX, NX);
            }
        }
        else
        {
            ThrowPretty("StartState has " << this->parameters_.StartState.rows() << " rows, but expected either NX=" << NX << " or NQ=" << num_positions_ << ", or NX*T=" << NX * T_);
        }
        set_X(start_state);
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(NX, NX);
    if (this->parameters_.Q.rows() > 0)
    {
        if (this->parameters_.Q.rows() == NX)
        {
            Q.diagonal() = this->parameters_.Q;
        }
        else
        {
            ThrowNamed("Q dimension mismatch! Expected " << NX << ", got " << this->parameters_.Q.rows());
        }
    }
    Q *= this->parameters_.Q_rate;
    Q_.assign(T_, Q);

    // Set final Q (Qf) -- the Qf variable has been populated in Instantiate
    set_Qf(Qf_);

    // Reinitialize general cost (via taskmaps)
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
    if (flags_ & KIN_J) jacobian.assign(T_, Eigen::MatrixXd(length_jacobian, N));
    if (flags_ & KIN_J_DOT)
    {
        Hessian Htmp;
        Htmp.setConstant(length_jacobian, Eigen::MatrixXd::Zero(N, N));
        hessian.assign(T_, Htmp);
    }
    cost.ReinitializeVariables(T_, shared_from_this(), cost_Phi);

    PreUpdate();
}

const int& DynamicTimeIndexedShootingProblem::get_T() const
{
    return T_;
}

void DynamicTimeIndexedShootingProblem::set_T(const int& T_in)
{
    if (T_in <= 2)
    {
        ThrowNamed("Invalid number of timesteps: " << T_in);
    }
    T_ = T_in;
    ReinitializeVariables();
}

const double& DynamicTimeIndexedShootingProblem::get_tau() const
{
    return tau_;
}

void DynamicTimeIndexedShootingProblem::PreUpdate()
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

    if (this->parameters_.WarmStartWithInverseDynamics)
    {
        for (int t = 0; t < T_ - 1; ++t)
        {
            U_.col(t) = scene_->GetDynamicsSolver()->InverseDynamics(X_.col(t));
            X_.col(t + 1) = scene_->GetDynamicsSolver()->Simulate(
                X_.col(t), U_.col(t), tau_);
        }
    }
}

const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::get_X() const
{
    return X_;
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::get_X(int t) const
{
    ValidateTimeIndex(t);
    return X_.col(t);
}

void DynamicTimeIndexedShootingProblem::set_X(Eigen::MatrixXdRefConst X_in)
{
    if (X_in.rows() != X_.rows() || X_in.cols() != X_.cols()) ThrowPretty("Sizes don't match! " << X_.rows() << "x" << X_.cols() << " vs " << X_in.rows() << "x" << X_in.cols());
    X_ = X_in;
}

const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::get_U() const
{
    return U_;
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::get_U(int t) const
{
    ValidateTimeIndex(t);
    return U_.col(t);
}

void DynamicTimeIndexedShootingProblem::set_U(Eigen::MatrixXdRefConst U_in)
{
    if (U_in.rows() != U_.rows() || U_in.cols() != U_.cols()) ThrowPretty("Sizes don't match! " << U_.rows() << "x" << U_.cols() << " vs " << U_in.rows() << "x" << U_in.cols());
    U_ = U_in;
}

const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::get_X_star() const
{
    return X_star_;
}

void DynamicTimeIndexedShootingProblem::set_X_star(Eigen::MatrixXdRefConst X_star_in)
{
    if (X_star_in.rows() != X_star_.rows() || X_star_in.cols() != X_star_.cols()) ThrowPretty("Sizes don't match! " << X_star_.rows() << "x" << X_star_.cols() << " vs " << X_star_in.rows() << "x" << X_star_in.cols());
    X_star_ = X_star_in;
}

const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::get_Q(int t) const
{
    ValidateTimeIndex(t);
    return Q_[t];
}

const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::get_Qf() const
{
    return Q_[T_ - 1];
}

const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::get_R() const
{
    return R_;
}

void DynamicTimeIndexedShootingProblem::set_Q(Eigen::MatrixXdRefConst Q_in, int t)
{
    ValidateTimeIndex(t);
    if (Q_in.rows() != Q_[t].rows() || Q_in.cols() != Q_[t].cols()) ThrowPretty("Dimension mismatch!");
    Q_[t] = Q_in;
}

void DynamicTimeIndexedShootingProblem::set_Qf(Eigen::MatrixXdRefConst Q_in)
{
    set_Q(Q_in, T_ - 1);
}

void DynamicTimeIndexedShootingProblem::Update(Eigen::VectorXdRefConst u_in, int t)
{
    // We can only update t=0, ..., T-1 - the last state will be created from integrating u_{T-1} to get x_T
    if (t >= (T_ - 1) || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }
    else if (t == -1)
    {
        t = T_ - 2;
    }

    if (u_in.rows() != num_controls_)
    {
        ThrowPretty("Mismatching in size of control vector: " << u_in.rows() << " given, expected: " << num_controls_);
    }

    U_.col(t) = u_in;

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

    // Simulate for tau
    X_.col(t + 1) = scene_->GetDynamicsSolver()->Simulate(X_.col(t), U_.col(t), tau_);

    // Stochstic noise, if enabled
    if (stochastic_matrices_specified_ && stochastic_updates_enabled_)
    {
        Eigen::VectorXd noise(num_positions_ + num_velocities_);
        for (int i = 0; i < num_positions_ + num_velocities_; ++i)
            noise(i) = standard_normal_noise_(generator_);

        Eigen::VectorXd control_dependent_noise = std::sqrt(scene_->GetDynamicsSolver()->get_dt()) * get_F(t) * noise;

        for (int i = 0; i < num_positions_ + num_velocities_; ++i)
            noise(i) = standard_normal_noise_(generator_);
        Eigen::VectorXd white_noise = std::sqrt(scene_->GetDynamicsSolver()->get_dt()) * CW_ * noise;

        X_.col(t + 1) = X_.col(t + 1) + white_noise + control_dependent_noise;
    }

    const Eigen::VectorXd x_next_position = scene_->GetDynamicsSolver()->GetPosition(X_.col(t + 1));
    scene_->Update(x_next_position, static_cast<double>(t) * tau_);

    Phi[t + 1].SetZero(length_Phi);
    if (flags_ & KIN_J) jacobian[t + 1].setZero();
    if (flags_ & KIN_J_DOT)
        for (int i = 0; i < length_jacobian; ++i) hessian[t + 1](i).setZero();
    for (int i = 0; i < num_tasks; ++i)
    {
        // Only update TaskMap if rho is not 0
        if (tasks_[i]->is_used)
        {
            if (flags_ & KIN_J_DOT)
            {
                tasks_[i]->Update(x_next_position, Phi[t + 1].data.segment(tasks_[i]->start, tasks_[i]->length), jacobian[t + 1].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian), hessian[t + 1].segment(tasks_[i]->start, tasks_[i]->length));
            }
            else if (flags_ & KIN_J)
            {
                tasks_[i]->Update(x_next_position, Phi[t + 1].data.segment(tasks_[i]->start, tasks_[i]->length), jacobian[t + 1].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian));
            }
            else
            {
                tasks_[i]->Update(x_next_position, Phi[t + 1].data.segment(tasks_[i]->start, tasks_[i]->length));
            }
        }
    }
    if (flags_ & KIN_J_DOT)
    {
        cost.Update(Phi[t + 1], jacobian[t + 1], hessian[t + 1], t + 1);
    }
    else if (flags_ & KIN_J)
    {
        cost.Update(Phi[t + 1], jacobian[t + 1], t + 1);
    }
    else
    {
        cost.Update(Phi[t + 1], t + 1);
    }

    ++number_of_problem_updates_;
}

double DynamicTimeIndexedShootingProblem::GetStateCost(int t) const
{
    ValidateTimeIndex(t);
    const Eigen::VectorXd x_diff = scene_->GetDynamicsSolver()->StateDelta(X_.col(t), X_star_.col(t));
    const double general_cost = cost.ydiff[t].transpose() * cost.S[t] * cost.ydiff[t];  // TODO: ct scaling
    return (x_diff.transpose() * Q_[t] * x_diff) + general_cost;
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::GetStateCostJacobian(int t) const
{
    // TODO: Check whether we should make this a RowVectorXd
    ValidateTimeIndex(t);
    const Eigen::VectorXd x_diff = scene_->GetDynamicsSolver()->StateDelta(X_.col(t), X_star_.col(t));
    const Eigen::VectorXd state_cost_jacobian = Q_[t] * x_diff + Q_[t].transpose() * x_diff;

    Eigen::VectorXd general_cost_jacobian = Eigen::VectorXd::Zero(num_positions_ + num_velocities_);
    general_cost_jacobian.head(num_positions_) = cost.jacobian[t].transpose() * cost.S[t] * cost.ydiff[t] * 2.0;

    return state_cost_jacobian + general_cost_jacobian;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::GetStateCostHessian(int t) const
{
    ValidateTimeIndex(t);
    Eigen::VectorXd general_cost_jacobian = Eigen::VectorXd::Zero(num_positions_ + num_velocities_);
    general_cost_jacobian.head(num_positions_) = cost.jacobian[t].transpose() * cost.S[t] * cost.ydiff[t] * 2.0;
    // TODO: Using a J^T*J approximation for the general cost here as Hessians aren't implemented for task maps yet.
    // TODO: As we are not using RowVectorXd (yet), this is J*J^T instead of the correct J^T*J
    return Q_[t] + Q_[t].transpose() + (general_cost_jacobian * general_cost_jacobian.transpose());
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::GetControlCostHessian() const
{
    return R_ + R_.transpose();
}

double DynamicTimeIndexedShootingProblem::GetControlCost(int t) const
{
    if (t >= T_ - 1 || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }
    else if (t == -1)
    {
        t = T_ - 2;
    }
    return U_.col(t).transpose() * R_ * U_.col(t);
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::GetControlCostJacobian(int t) const
{
    if (t >= T_ - 1 || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }
    else if (t == -1)
    {
        t = T_ - 2;
    }
    return R_ * U_.col(t) + R_.transpose() * U_.col(t);
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::Dynamics(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u)
{
    return scene_->GetDynamicsSolver()->f(x, u);
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::Simulate(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u)
{
    return scene_->GetDynamicsSolver()->Simulate(x, u, tau_);
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::get_F(int t) const
{
    if (t >= T_ - 1 || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }

    const int NX = num_positions_ + num_velocities_;
    Eigen::MatrixXd F(NX, NX);

    for (int i = 0; i < NX; ++i)
        F.col(i) = Ci_[i] * U_.col(t);

    return F;
}

// F[i]_u
const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::GetControlNoiseJacobian(int column_idx) const
{
    if (column_idx < 0 || column_idx >= num_velocities_)
        ThrowPretty("Requested column_idx=" << column_idx << " out of range; needs to be 0 <= column_idx < " << num_velocities_ - 1);
    return Ci_[column_idx];
}

void DynamicTimeIndexedShootingProblem::EnableStochasticUpdates()
{
    stochastic_updates_enabled_ = true;
}

void DynamicTimeIndexedShootingProblem::DisableStochasticUpdates()
{
    stochastic_updates_enabled_ = false;
}

}  // namespace exotica
