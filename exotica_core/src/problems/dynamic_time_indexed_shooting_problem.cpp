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
#include <exotica_core/tools/conversions.h>
#include <exotica_core/tools/sparse_costs.h>
#include <cmath>

REGISTER_PROBLEM_TYPE("DynamicTimeIndexedShootingProblem", exotica::DynamicTimeIndexedShootingProblem)

namespace exotica
{
DynamicTimeIndexedShootingProblem::DynamicTimeIndexedShootingProblem()
{
    this->flags_ = KIN_FK | KIN_J;
}
DynamicTimeIndexedShootingProblem::~DynamicTimeIndexedShootingProblem() = default;

void DynamicTimeIndexedShootingProblem::InstantiateCostTerms(const DynamicTimeIndexedShootingProblemInitializer& init)
{
    // L1 Rate
    if (parameters_.LossType == "SmoothL1")
    {
        if (parameters_.L1Rate.size() == 0)
        {
            ThrowPretty("L1Rate not set.");  // TODO: set default...
        }
        else if (parameters_.L1Rate.size() == 1)
        {
            l1_rate_.setConstant(scene_->get_num_controls(), parameters_.L1Rate(0));
        }
        else if (parameters_.L1Rate.size() == scene_->get_num_controls())
        {
            l1_rate_ = parameters_.L1Rate;
        }
        else
        {
            ThrowPretty("L1Rate has wrong size: expected " << scene_->get_num_controls() << ", got " << parameters_.L1Rate.size());
        }
    }

    // Huber Rate
    if (parameters_.LossType == "Huber" || parameters_.LossType == "SuperHuber" || parameters_.LossType == "BiModalHuber")
    {
        if (parameters_.HuberRate.size() == 0)
        {
            ThrowPretty("HuberRate not set.");  // TODO: set default...
        }
        else if (parameters_.HuberRate.size() == 1)
        {
            huber_rate_.setConstant(scene_->get_num_controls(), parameters_.HuberRate(0));
        }
        else if (parameters_.HuberRate.size() == scene_->get_num_controls())
        {
            huber_rate_ = parameters_.HuberRate;
        }
        else
        {
            ThrowPretty("HuberRate has wrong size: expected " << scene_->get_num_controls() << ", got " << parameters_.HuberRate.size());
        }
    }

    // BimodalHuber
    if (parameters_.LossType == "BiModalHuber")
    {
        // BimodalHuber mode 1
        if (parameters_.Mode1.size() == 0)
        {
            ThrowPretty("Mode1 not set.");  // TODO: set default...
        }
        else if (parameters_.Mode1.size() == 1)
        {
            bimodal_huber_mode1_.setConstant(scene_->get_num_controls(), parameters_.Mode1(0));
        }
        else if (parameters_.Mode1.size() == scene_->get_num_controls())
        {
            bimodal_huber_mode1_ = parameters_.Mode1;
        }
        else
        {
            ThrowPretty("Mode1 has wrong size: expected " << scene_->get_num_controls() << ", got " << parameters_.Mode1.size());
        }

        // BimodalHuber mode 1
        if (parameters_.Mode2.size() == 0)
        {
            ThrowPretty("Mode2 not set.");  // TODO: set default...
        }
        else if (parameters_.Mode2.size() == 1)
        {
            bimodal_huber_mode2_.setConstant(scene_->get_num_controls(), parameters_.Mode2(0));
        }
        else if (parameters_.Mode2.size() == scene_->get_num_controls())
        {
            bimodal_huber_mode2_ = parameters_.Mode2;
        }
        else
        {
            ThrowPretty("Mode2 has wrong size: expected " << scene_->get_num_controls() << ", got " << parameters_.Mode2.size());
        }
    }
    control_cost_weight_ = parameters_.ControlCostWeight;
}

void DynamicTimeIndexedShootingProblem::Instantiate(const DynamicTimeIndexedShootingProblemInitializer& init)
{
    this->parameters_ = init;

    if (!scene_->GetDynamicsSolver()) ThrowPretty("DynamicsSolver is not initialised!");

    const int NX = scene_->get_num_positions() + scene_->get_num_velocities(),
              NDX = 2 * scene_->get_num_velocities(),
              NU = scene_->get_num_controls();
    Qf_ = Eigen::MatrixXd::Identity(NDX, NDX);
    if (this->parameters_.Qf.rows() > 0)
    {
        if (this->parameters_.Qf.rows() == NDX)
        {
            Qf_.diagonal() = this->parameters_.Qf;
        }
        else
        {
            ThrowNamed("Qf dimension mismatch! Expected " << NDX << ", got " << this->parameters_.Qf.rows());
        }
    }
    Qf_ *= this->parameters_.Qf_rate;

    R_ = this->parameters_.R_rate * Eigen::MatrixXd::Identity(scene_->get_num_controls(), scene_->get_num_controls());
    if (this->parameters_.R.rows() > 0)
    {
        if (this->parameters_.R.rows() == scene_->get_num_controls())
        {
            R_.diagonal() = this->parameters_.R;
        }
        else
        {
            ThrowNamed("R dimension mismatch! Expected " << scene_->get_num_controls() << ", got " << this->parameters_.R.rows());
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
    InstantiateCostTerms(init);
    ReinitializeVariables();
}

void DynamicTimeIndexedShootingProblem::ReinitializeVariables()
{
    if (debug_) HIGHLIGHT_NAMED("DynamicTimeIndexedShootingProblem", "Initialize problem with T=" << T_);

    const int NX = scene_->get_num_positions() + scene_->get_num_velocities(), NDX = 2 * scene_->get_num_velocities();

    X_ = Eigen::MatrixXd::Zero(NX, T_);
    X_star_ = Eigen::MatrixXd::Zero(NX, T_);
    X_diff_ = Eigen::MatrixXd::Zero(NDX, T_);
    U_ = Eigen::MatrixXd::Zero(scene_->get_num_controls(), T_ - 1);

    // Set w component of quaternion by default
    if (scene_->get_has_quaternion_floating_base())
    {
        for (int t = 0; t < T_; ++t)
        {
            SetDefaultQuaternionInConfigurationVector(X_.col(t));
            SetDefaultQuaternionInConfigurationVector(X_star_.col(t));
        }
    }

    // Set GoalState
    if (this->parameters_.GoalState.rows() > 0)
    {
        Eigen::MatrixXd goal_state(X_star_);

        if (this->parameters_.GoalState.rows() == NX)
        {
            goal_state.col(T_ - 1) = this->parameters_.GoalState;
        }
        else if (this->parameters_.GoalState.rows() == scene_->get_num_positions())
        {
            goal_state.col(T_ - 1).head(scene_->get_num_positions()) = this->parameters_.GoalState;
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
            ThrowPretty("GoalState has " << this->parameters_.GoalState.rows() << " rows, but expected either NX=" << NX << " or NQ=" << scene_->get_num_positions() << ", or NX*T=" << NX * T_);
        }
        set_X_star(goal_state);
    }

    // Set StartState
    if (this->parameters_.StartState.rows() > 0)
    {
        Eigen::MatrixXd start_state(X_);
        if (this->parameters_.StartState.rows() == NX)
        {
            start_state = this->parameters_.StartState.replicate(1, T_);
        }
        else if (this->parameters_.StartState.rows() == scene_->get_num_positions())
        {
            for (int t = 0; t < T_; ++t)
            {
                start_state.col(t).head(scene_->get_num_positions()) = this->parameters_.StartState;
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
            ThrowPretty("StartState has " << this->parameters_.StartState.rows() << " rows, but expected either NX=" << NX << " or NQ=" << scene_->get_num_positions() << ", or NX*T=" << NX * T_);
        }
        set_X(start_state);
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(NDX, NDX);
    if (this->parameters_.Q.rows() > 0)
    {
        if (this->parameters_.Q.rows() == NDX)
        {
            Q.diagonal() = this->parameters_.Q;
        }
        else
        {
            ThrowNamed("Q dimension mismatch! Expected " << NDX << ", got " << this->parameters_.Q.rows());
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

    // Initialize the TaskSpaceVector and its derivatives
    y_ref_.SetZero(length_Phi);
    Phi.assign(T_, y_ref_);

    if (flags_ & KIN_J)
    {
        dPhi_dx.assign(T_, Eigen::MatrixXd(length_jacobian, scene_->get_num_state_derivative()));
        dPhi_du.assign(T_, Eigen::MatrixXd(length_jacobian, scene_->get_num_controls()));
    }

    if (flags_ & KIN_H)
    {
        ddPhi_ddx.assign(T_, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(scene_->get_num_state_derivative(), scene_->get_num_state_derivative())));
        ddPhi_ddu.assign(T_, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(scene_->get_num_controls(), scene_->get_num_controls())));
        ddPhi_dxdu.assign(T_, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(scene_->get_num_state_derivative(), scene_->get_num_controls())));
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
            X_.col(t + 1) = scene_->GetDynamicsSolver()->Simulate(X_.col(t), U_.col(t), tau_);
        }
    }
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::ApplyStartState(bool update_traj)
{
    PlanningProblem::ApplyStartState(update_traj);
    return start_state_;
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

    // Normalize quaternion, if required.
    if (scene_->get_has_quaternion_floating_base())
    {
        for (int t = 0; t < T_; ++t)
        {
            NormalizeQuaternionInConfigurationVector(X_.col(t));
        }
    }
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

    // Normalize quaternion, if required.
    if (scene_->get_has_quaternion_floating_base())
    {
        for (int t = 0; t < T_; ++t)
        {
            NormalizeQuaternionInConfigurationVector(X_star_.col(t));
        }
    }
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

void DynamicTimeIndexedShootingProblem::Update(Eigen::VectorXdRefConst x_in, Eigen::VectorXdRefConst u_in, int t)
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

    if (u_in.rows() != scene_->get_num_controls())
    {
        ThrowPretty("Mismatching in size of control vector: " << u_in.rows() << " given, expected: " << scene_->get_num_controls());
    }

    if (x_in.rows() != scene_->get_num_positions() + scene_->get_num_velocities())
    {
        ThrowPretty("Mismatching in size of state vector vector: " << x_in.rows() << " given, expected: " << scene_->get_num_positions() + scene_->get_num_velocities());
    }

    X_.col(t) = x_in;
    U_.col(t) = u_in;

    // Update xdiff
    X_diff_.col(t) = scene_->GetDynamicsSolver()->StateDelta(X_.col(t), X_star_.col(t));

    // Update current state kinematics and costs
    UpdateTaskMaps(X_.col(t), U_.col(t), t);

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

    // Update xdiff
    X_diff_.col(t + 1) = scene_->GetDynamicsSolver()->StateDelta(X_.col(t + 1), X_star_.col(t + 1));

    // Stochastic noise, if enabled
    if (stochastic_matrices_specified_ && stochastic_updates_enabled_)
    {
        Eigen::VectorXd noise(scene_->get_num_positions() + scene_->get_num_velocities());
        for (int i = 0; i < scene_->get_num_positions() + scene_->get_num_velocities(); ++i)
            noise(i) = standard_normal_noise_(generator_);

        Eigen::VectorXd control_dependent_noise = std::sqrt(scene_->GetDynamicsSolver()->get_dt()) * get_F(t) * noise;

        for (int i = 0; i < scene_->get_num_positions() + scene_->get_num_velocities(); ++i)
            noise(i) = standard_normal_noise_(generator_);
        Eigen::VectorXd white_noise = std::sqrt(scene_->GetDynamicsSolver()->get_dt()) * CW_ * noise;

        X_.col(t + 1) = X_.col(t + 1) + white_noise + control_dependent_noise;
    }

    // TODO: We are now updating the TaskMaps _twice_ per call. This may be very expensive (!)
    // const Eigen::VectorXd q_next_position = scene_->GetDynamicsSolver()->GetPosition(X_.col(t + 1));
    // UpdateTaskMaps(q_next_position, t + 1);

    ++number_of_problem_updates_;
}

void DynamicTimeIndexedShootingProblem::Update(Eigen::VectorXdRefConst u, int t)
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

    return Update(X_.col(t), u, t);
}

void DynamicTimeIndexedShootingProblem::UpdateTerminalState(Eigen::VectorXdRefConst x_in)
{
    int t = T_ - 1;

    if (x_in.rows() != scene_->get_num_positions() + scene_->get_num_velocities())
    {
        ThrowPretty("Mismatching in size of state vector vector: " << x_in.rows() << " given, expected: " << scene_->get_num_positions() + scene_->get_num_velocities());
    }

    X_.col(t) = x_in;
    X_diff_.col(t) = scene_->GetDynamicsSolver()->StateDelta(X_.col(t), X_star_.col(t));

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

    UpdateTaskMaps(X_.col(t), Eigen::VectorXd::Zero(scene_->get_num_controls()), t);

    ++number_of_problem_updates_;
}

void DynamicTimeIndexedShootingProblem::UpdateTaskMaps(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, int t)
{
    ValidateTimeIndex(t);

    // Update the kinematic scene based on the configuration.
    // NB: The KinematicTree only understands a certain format for the configuration (RPY)
    // => As a result, we need to use GetPosition to potentially convert.
    const Eigen::VectorXd q = scene_->GetDynamicsSolver()->GetPosition(x);
    scene_->Update(q, static_cast<double>(t) * tau_);

    // Reset the task space vector and its derivatives for the current timestep
    Phi[t].SetZero(length_Phi);

    if (flags_ & KIN_J)
    {
        dPhi_dx[t].setZero();
        dPhi_du[t].setZero();
    }

    if (flags_ & KIN_H)
    {
        for (int i = 0; i < length_jacobian; ++i)
        {
            ddPhi_ddx[t](i).setZero();
            ddPhi_ddu[t](i).setZero();
            ddPhi_dxdu[t](i).setZero();
        }
    }

    // Update all task-maps
    for (int i = 0; i < num_tasks; ++i)
    {
        // Only update TaskMap if rho is not 0
        if (tasks_[i]->is_used)
        {
            if (flags_ & KIN_H)
            {
                tasks_[i]->Update(x, u,
                                  Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length),
                                  dPhi_dx[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian),
                                  dPhi_du[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian),
                                  ddPhi_ddx[t].segment(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian),
                                  ddPhi_ddu[t].segment(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian),
                                  ddPhi_dxdu[t].segment(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian));
            }
            else if (flags_ & KIN_J)
            {
                tasks_[i]->Update(x, u,
                                  Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length),
                                  dPhi_dx[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian),
                                  dPhi_du[t].middleRows(tasks_[i]->start_jacobian, tasks_[i]->length_jacobian));
            }
            else
            {
                tasks_[i]->Update(x, u, Phi[t].data.segment(tasks_[i]->start, tasks_[i]->length));
            }
        }
    }

    // Update costs (TimeIndexedTask)
    if (flags_ & KIN_H)
    {
        cost.Update(Phi[t], dPhi_dx[t], dPhi_du[t], ddPhi_ddx[t], ddPhi_ddu[t], ddPhi_dxdu[t], t);
    }
    else if (flags_ & KIN_J)
    {
        cost.Update(Phi[t], dPhi_dx[t], dPhi_du[t], t);
    }
    else
    {
        cost.Update(Phi[t], t);
    }
}

double DynamicTimeIndexedShootingProblem::GetStateCost(int t) const
{
    ValidateTimeIndex(t);
    const double state_cost = X_diff_.col(t).transpose() * Q_[t] * X_diff_.col(t);
    const double general_cost = cost.ydiff[t].transpose() * cost.S[t] * cost.ydiff[t];
    return state_cost + general_cost;  // TODO: ct scaling
}

Eigen::RowVectorXd DynamicTimeIndexedShootingProblem::GetStateCostJacobian(int t) const
{
    ValidateTimeIndex(t);

    // (1,1) * (NDX,1)^T * (NDX,NDX) * (NDX,NDX) => (1,NDX)
    const Eigen::MatrixXd dxdiff = scene_->GetDynamicsSolver()->dStateDelta(X_.col(t), X_star_.col(t), ArgumentPosition::ARG0);
    const Eigen::RowVectorXd state_cost_jacobian = 2.0 * X_diff_.col(t).transpose() * Q_[t] * dxdiff;

    // m => dimension of task maps, "length_jacobian"
    // (1,1) * (m,1)^T * (m,m) * (m,NQ) => (1,NQ)
    Eigen::RowVectorXd general_cost_jacobian = 2.0 * cost.ydiff[t].transpose() * cost.S[t] * cost.dPhi_dx[t];

    return state_cost_jacobian + general_cost_jacobian;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::GetStateCostHessian(int t) const
{
    ValidateTimeIndex(t);
    const int ndx = 2 * scene_->get_num_velocities();

    // State Cost
    Eigen::MatrixXd state_cost_hessian = Eigen::MatrixXd::Zero(ndx, ndx);
    const Eigen::MatrixXd dxdiff = scene_->GetDynamicsSolver()->dStateDelta(X_.col(t), X_star_.col(t), ArgumentPosition::ARG0);
    state_cost_hessian.noalias() = dxdiff.transpose() * Q_[t] * dxdiff;

    // For non-Euclidean spaces (i.e. on manifolds), there exists a second derivative of the state delta
    if (scene_->get_has_quaternion_floating_base())
    {
        Eigen::RowVectorXd xdiffTQ = X_diff_.col(t).transpose() * Q_[t];  // (1*ndx)
        Hessian ddxdiff = scene_->GetDynamicsSolver()->ddStateDelta(X_.col(t), X_star_.col(t), ArgumentPosition::ARG0);
        for (int i = 0; i < ndx; ++i)
        {
            state_cost_hessian.noalias() += xdiffTQ(i) * ddxdiff(i);
        }
    }

    // General Cost
    Eigen::MatrixXd general_cost_hessian = Eigen::MatrixXd::Zero(ndx, ndx);
    general_cost_hessian.noalias() = cost.dPhi_dx[t].transpose() * cost.S[t] * cost.dPhi_dx[t];

    // Contract task-map Hessians
    if (flags_ & KIN_H)
    {
        Eigen::RowVectorXd ydiffTS = cost.ydiff[t].transpose() * cost.S[t];  // (1*m)
        for (int i = 0; i < cost.length_jacobian; ++i)                       // length m
        {
            general_cost_hessian.noalias() += ydiffTS(i) * cost.ddPhi_ddx[t](i);
        }
    }

    return 2.0 * state_cost_hessian + 2.0 * general_cost_hessian;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::GetControlCostHessian(int t) const
{
    if (t >= T_ - 1 || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }
    else if (t == -1)
    {
        t = T_ - 2;
    }

    // auto dynamics_solver = scene_->GetDynamicsSolver();
    // auto control_limits = dynamics_solver->get_control_limits();

    // Sparsity-related control Hessian
    Eigen::MatrixXd Quu = Eigen::MatrixXd::Zero(scene_->get_num_controls(), scene_->get_num_controls());

    // This allows composition of multiple functions
    //  useful when you want to apply different cost functions to different controls
    // if (parameters_.LossType == "L2")
    Quu += R_ + R_.transpose();

    for (int iu = 0; iu < scene_->get_num_controls(); ++iu)
    {
        if (parameters_.LossType == "SmoothL1")
            Quu(iu, iu) += smooth_l1_hessian(U_.col(t)[iu], l1_rate_(iu));

        else if (parameters_.LossType == "SuperHuber")
            Quu(iu, iu) += super_huber_hessian(U_.col(t)[iu], huber_rate_(iu), parameters_.SuperHuberFactor);

        // if huber_rate is 0, huber is undefined
        //  this is a shortcut for disabling the loss
        else if (parameters_.LossType == "Huber" && huber_rate_(iu) != 0)
            Quu(iu, iu) += huber_hessian(U_.col(t)[iu], huber_rate_(iu));

        else if (parameters_.LossType == "BiModalHuber" && huber_rate_(iu) != 0)
            Quu(iu, iu) += bimodal_huber_hessian(
                U_.col(t)[iu], huber_rate_(iu), bimodal_huber_mode1_(iu), bimodal_huber_mode2_(iu));
    }
    return control_cost_weight_ * Quu;
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

    // auto dynamics_solver = scene_->GetDynamicsSolver();
    // auto control_limits = dynamics_solver->get_control_limits();

    // Sparsity-related control cost
    double cost = 0;

    // This allows composition of multiple functions
    //  useful when you want to apply different cost functions to different controls
    // if (parameters_.LossType == "L2")
    cost += U_.col(t).transpose() * R_ * U_.col(t);

    for (int iu = 0; iu < scene_->get_num_controls(); ++iu)
    {
        // if (U_.col(t)[iu] >= control_limits.col(1)[iu])
        //     continue;
        if (parameters_.LossType == "SmoothL1")
            cost += smooth_l1_cost(U_.col(t)[iu], l1_rate_(iu));

        // if huber_rate is 0, huber is undefined
        //  this is a shortcut for disabling the loss
        else if (parameters_.LossType == "Huber" && huber_rate_(iu) != 0)
            cost += huber_cost(U_.col(t)[iu], huber_rate_(iu));

        else if (parameters_.LossType == "SuperHuber")
            cost += super_huber_cost(U_.col(t)[iu], huber_rate_(iu), parameters_.SuperHuberFactor);

        else if (parameters_.LossType == "BiModalHuber" && huber_rate_(iu) != 0)
            cost += bimodal_huber_cost(
                U_.col(t)[iu], huber_rate_(iu), bimodal_huber_mode1_(iu), bimodal_huber_mode2_(iu));
    }
    if (!std::isfinite(cost))
    {
        cost = 0.0;  // Likely "inf" as u is too small.
    }
    return control_cost_weight_ * cost;
}

Eigen::RowVectorXd DynamicTimeIndexedShootingProblem::GetControlCostJacobian(int t) const
{
    if (t >= T_ - 1 || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }
    else if (t == -1)
    {
        t = T_ - 2;
    }

    // Sparsity-related control cost Jacobian
    Eigen::RowVectorXd Qu = Eigen::RowVectorXd::Zero(scene_->get_num_controls());

    // This allows composition of multiple functions
    //  useful when you want to apply different cost functions to different controls
    // if (parameters_.LossType == "L2")
    Qu += 2.0 * U_.col(t).transpose() * R_;  // Assumes R is diagonal

    for (int iu = 0; iu < scene_->get_num_controls(); ++iu)
    {
        // if (U_.col(t)[iu] >= control_limits.col(1)[iu])
        //     continue;
        if (parameters_.LossType == "SmoothL1")
            Qu(iu) += smooth_l1_jacobian(U_.col(t)[iu], l1_rate_(iu));

        // if huber_rate is 0, huber is undefined
        //  this is a shortcut for disabling the loss
        else if (parameters_.LossType == "Huber" && huber_rate_(iu) != 0)
            Qu(iu) += huber_jacobian(U_.col(t)[iu], huber_rate_(iu));

        else if (parameters_.LossType == "BiModalHuber" && huber_rate_(iu) != 0)
            Qu(iu) += bimodal_huber_jacobian(
                U_.col(t)[iu], huber_rate_(iu), bimodal_huber_mode1_(iu), bimodal_huber_mode2_(iu));
        else if (parameters_.LossType == "SuperHuber")
            Qu(iu) += super_huber_jacobian(U_.col(t)[iu], huber_rate_(iu), parameters_.SuperHuberFactor);
    }
    return control_cost_weight_ * Qu;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::get_F(int t) const
{
    if (t >= T_ - 1 || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_ - 1);
    }

    const int NX = scene_->get_num_positions() + scene_->get_num_velocities();
    Eigen::MatrixXd F(NX, NX);

    for (int i = 0; i < NX; ++i)
        F.col(i) = Ci_[i] * U_.col(t);

    return F;
}

// F[i]_u
const Eigen::MatrixXd& DynamicTimeIndexedShootingProblem::GetControlNoiseJacobian(int column_idx) const
{
    if (column_idx < 0 || column_idx >= scene_->get_num_velocities())
        ThrowPretty("Requested column_idx=" << column_idx << " out of range; needs to be 0 <= column_idx < " << scene_->get_num_velocities() - 1);
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
