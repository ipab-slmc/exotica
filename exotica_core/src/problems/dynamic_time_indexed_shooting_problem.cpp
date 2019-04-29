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
DynamicTimeIndexedShootingProblem::DynamicTimeIndexedShootingProblem() = default;

DynamicTimeIndexedShootingProblem::~DynamicTimeIndexedShootingProblem() = default;

void DynamicTimeIndexedShootingProblem::Instantiate(DynamicTimeIndexedShootingProblemInitializer& init)
{
    init_ = init;

    // Create dynamics solver
    dynamics_solver_ = Setup::CreateDynamicsSolver(init_.DynamicsSolver);
    dynamics_solver_->AssignScene(scene_);
    dynamics_solver_->SetDt(init_.dt);
    dynamics_solver_->SetIntegrator(init_.Integrator);

    // TODO: Strictly speaking N here should correspond to the number of controls, which comes from the dynamic solver - to be fixed!
    N = scene_->GetKinematicTree().GetNumControlledJoints();
    num_positions_ = dynamics_solver_->get_num_positions();
    num_velocities_ = dynamics_solver_->get_num_velocities();
    num_controls_ = dynamics_solver_->get_num_controls();

    const int NX = num_positions_ + num_velocities_;
    if (init_.Q.rows() > 0)
    {
        ThrowPretty("Not supported yet");
        // if (init_.Q.rows() == NX)
        // {
        //     Q_.diagonal() = init_.Q;
        // }
        // else
        // {
        //     ThrowNamed("Q dimension mismatch! Expected " << NX << ", got " << init_.Q.rows());
        // }
    }

    R_ = init_.R_rate * Eigen::MatrixXd::Identity(num_controls_, num_controls_);
    if (init_.R.rows() > 0)
    {
        if (init_.R.rows() == num_controls_)
        {
            R_.diagonal() = init_.R;
        }
        else
        {
            ThrowNamed("R dimension mismatch! Expected " << num_controls_ << ", got " << init_.R.rows());
        }
    }

    T_ = init_.T;
    tau_ = init_.tau;

    // For now, without inter-/extra-polation for integrators, assure that tau is a multiple of dt
    const double fmod_tau_dt = std::fmod(static_cast<long double>(1000. * tau_), static_cast<long double>(1000. * init_.dt));
    if (fmod_tau_dt > 1e-5) ThrowPretty("tau is not a multiple of dt: tau=" << tau_ << ", dt=" << init_.dt << ", mod(" << fmod_tau_dt << ")");

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

    Q_.assign(T_, init_.Q_rate * Eigen::MatrixXd::Identity(NX, NX));

    PreUpdate();
}

int DynamicTimeIndexedShootingProblem::get_T() const
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

double DynamicTimeIndexedShootingProblem::get_tau() const
{
    return tau_;
}

void DynamicTimeIndexedShootingProblem::PreUpdate()
{
    PlanningProblem::PreUpdate();
    for (int i = 0; i < tasks_.size(); ++i) tasks_[i]->is_used = false;

    // Create a new set of kinematic solutions with the size of the trajectory
    // based on the lastest KinematicResponse in order to reflect model state
    // updates etc.
    kinematic_solutions_.clear();
    kinematic_solutions_.resize(T_);
    for (int i = 0; i < T_; ++i) kinematic_solutions_[i] = std::make_shared<KinematicResponse>(*scene_->GetKinematicTree().GetKinematicResponse());
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::get_X() const
{
    return X_;
}

void DynamicTimeIndexedShootingProblem::set_X(Eigen::MatrixXdRefConst X_in)
{
    if (X_in.rows() != X_.rows() || X_in.cols() != X_.cols()) ThrowPretty("Sizes don't match!");
    X_ = X_in;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::get_U() const
{
    return U_;
}

void DynamicTimeIndexedShootingProblem::set_U(Eigen::MatrixXdRefConst U_in)
{
    if (U_in.rows() != U_.rows() || U_in.cols() != U_.cols()) ThrowPretty("Sizes don't match!");
    U_ = U_in;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::get_X_star() const
{
    return X_star_;
}

void DynamicTimeIndexedShootingProblem::set_X_star(Eigen::MatrixXdRefConst X_star_in)
{
    if (X_star_in.rows() != X_star_.rows() || X_star_in.cols() != X_star_.cols()) ThrowPretty("Sizes don't match!");
    X_star_ = X_star_in;
}

Eigen::MatrixXd DynamicTimeIndexedShootingProblem::get_Q(int t) const
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    return Q_[t];
}

void DynamicTimeIndexedShootingProblem::set_Q(Eigen::MatrixXdRefConst Q_in, int t)
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    if (Q_in.rows() != Q_[t].rows() || Q_in.cols() != Q_[t].cols()) ThrowPretty("Dimension mismatch!");
    Q_[t] = Q_in;
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
    X_.col(t + 1) = dynamics_solver_->Simulate(X_.col(t), U_.col(t), tau_);

    scene_->Update(dynamics_solver_->GetPosition(X_.col(t + 1)), static_cast<double>(t) * tau_);

    // TODO: Cost, Equality, Inequality

    ++number_of_problem_updates_;
}

double DynamicTimeIndexedShootingProblem::GetStateCost(int t) const
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    const Eigen::VectorXd x_diff = X_star_.col(t) - X_.col(t);
    return (x_diff.transpose() * Q_[t] * x_diff);
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::GetStateCostJacobian(int t) const
{
    if (t >= T_ || t < -1)
    {
        ThrowPretty("Requested t=" << t << " out of range, needs to be 0 =< t < " << T_);
    }
    else if (t == -1)
    {
        t = T_ - 1;
    }
    const Eigen::VectorXd x_diff = X_star_.col(t) - X_.col(t);
    return x_diff.transpose() * Q_[t] * dynamics_solver_->fu(X_.col(t), U_.col(t)) * -2.0;
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
    return U_.col(t) * R_ * 2.0;
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::Dynamics(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u)
{
    return dynamics_solver_->f(x, u);
}

Eigen::VectorXd DynamicTimeIndexedShootingProblem::Simulate(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u)
{
    return dynamics_solver_->Simulate(x, u, tau_);
}

// min (mu-x)^T * Q * (mu-x) + u^T * R * u

}  // namespace exotica
