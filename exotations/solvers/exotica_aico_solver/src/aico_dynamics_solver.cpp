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

// This code is based on algorithm developed by Marc Toussaint
// M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
// http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
// Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html

/// \file aico_dynamics_solver.cpp
/// \brief Approximate Inference Control

#include <exotica_aico_solver/aico_dynamics_solver.h>
// #include <exotica_core/server.h>

REGISTER_MOTIONSOLVER_TYPE("AICODynamicsSolver", exotica::AICODynamicsSolver)

namespace exotica
{

void AICODynamicsSolver::SpecifyProblem(PlanningProblemPtr pointer)
{
    if (pointer->type() != "exotica::DynamicTimeIndexedShootingProblem")
    {
        ThrowNamed("This AICODynamicsSolver can't solve problem of type '" << pointer->type() << "'!");
    }
    MotionSolver::SpecifyProblem(pointer);
    prob_ = std::static_pointer_cast<DynamicTimeIndexedShootingProblem>(pointer);
    dynamics_solver_ = prob_->GetScene()->GetDynamicsSolver();
    if (debug_) HIGHLIGHT_NAMED("AICODynamicsSolver", "initialized");
}


void AICODynamicsSolver::Solve(Eigen::MatrixXd& solution)
{
    if (!prob_) ThrowNamed("Solver has not been initialized!");
    prob_->ResetCostEvolution(GetNumberOfMaxIterations() + 1);

    Timer solution_timer;

    Sweep(solution);

    HIGHLIGHT_NAMED("AICODynamicsSolver", "Solved! Time: " << solution_timer.GetDuration());
}

void AICODynamicsSolver::Sweep(Eigen::MatrixXd& solution)
{
    const int T = prob_->get_T();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();
    const int NQ = prob_->get_num_positions();
    const double dt = dynamics_solver_->get_dt();
    const double alpha = parameters_.alpha_rate;
    const double theta = parameters_.theta_rate;
    double current_cost = 0,
        last_cost = 1e5,
        global_best_cost = 1e5;
    int last_best_iteration = 0;
    
    solution.resize(T, NU);

    // s matrices
    s_.assign(T, prob_->get_X(0));
    Sinv_.assign(T, Eigen::MatrixXd::Zero(NX, NX));
    Sinv_[0].diagonal().setConstant(1e10);

    S_.assign(T, Eigen::MatrixXd::Zero(NX, NX));
    S_[0].diagonal().setConstant(1e-10);

    v_.assign(T, Eigen::VectorXd::Zero(NX));
    V_.assign(T, Eigen::MatrixXd::Zero(NX, NX));
    Vinv_.assign(T, Eigen::MatrixXd::Zero(NX, NX));

    R_.assign(T, Eigen::MatrixXd::Zero(NX, NX));
    r_.assign(T, Eigen::VectorXd::Zero(NX));

    a_.assign(T, Eigen::VectorXd::Zero(NX));
    A_.assign(T, Eigen::MatrixXd::Zero(NX, NX));
    B_.assign(T, Eigen::MatrixXd::Zero(NX, NU));

    // belief
    b_.assign(T, Eigen::VectorXd::Zero(NX));

    rhat = Eigen::VectorXd::Zero(T);
    qhat.assign(T, Eigen::VectorXd::Zero(NX));

    // Hurr-durr
    Eigen::MatrixXd H = prob_->get_R();
    Eigen::MatrixXd Hinv = (H + Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-5).inverse();

    Xhat_ = prob_->get_X();

    // forward & backwards loops
    for (int iteration = 0; iteration < GetNumberOfMaxIterations(); ++iteration)
    {
        Timer planning_timer;
        UpdateProcess(prob_->get_X(0), prob_->get_U(0), 0);
        prob_->set_X(Xhat_);

        // forward loop
        for (int t = 1; t < T; ++ t)
        {
            if (iteration == 0) Xhat_.col(t) = s_[t];
            else Xhat_.col(t) = (1 - alpha) * Xhat_.col(t) + alpha * b_[t];
            prob_->set_X(Xhat_);

            UpdateProcess(prob_->get_X(t), prob_->get_U(t), t);

            Eigen::MatrixXd Atinv = (A_[t] + Eigen::MatrixXd::Identity(A_[t].rows(), A_[t].cols()) * 1e-5).inverse();

            // TODO: Add white noise
            Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(NX, NX);
            if (t < T - 1) Q = prob_->get_F(t);

            Eigen::MatrixXd SplusRinv = (Sinv_[t - 1] + R_[t - 1] + Eigen::MatrixXd::Identity(R_[t - 1].rows(), R_[t - 1].cols()) * 1e-5).inverse();

            s_[t] = a_[t - 1] + A_[t - 1] * SplusRinv * (Sinv_[t - 1] * s_[t - 1] + r_[t - 1]);
            S_[t] = Q + B_[t - 1] * Hinv * B_[t - 1].transpose() + A_[t - 1] * SplusRinv * A_[t - 1].transpose();
            Sinv_[t] = (S_[t] + Eigen::MatrixXd::Identity(S_[t].rows(), S_[t].cols()) * 1e-5).inverse();

            if (t < T - 1)
            {
                Eigen::MatrixXd VplusRinv = (Vinv_[t + 1] + R_[t + 1] + Eigen::MatrixXd::Identity(R_[t + 1].rows(), R_[t + 1].cols()) * 1e-5).inverse();
                v_[t] = -Atinv * a_[t] + Atinv * VplusRinv * (Vinv_[t + 1] * v_[t + 1] + r_[t + 1]);
                V_[t] = -Atinv * (Q + B_[t] * Hinv * B_[t].transpose() + VplusRinv) * Atinv.transpose();
                Vinv_[t] = (V_[t] + Eigen::MatrixXd::Identity(V_[t].rows(), V_[t].cols()) * 1e-5).inverse();
            }

            Eigen::MatrixXd Binv = Sinv_[t] + Vinv_[t] + R_[t];
            AinvBSymPosDef(b_[t], Binv, Sinv_[t] * s_[t] + Vinv_[t] * v_[t] + r_[t]);

            if ((b_[t] - Xhat_.col(t)).squaredNorm() > theta)
            {
                t = t - 1;
                // HIGHLIGHT_NAMED("AICODynamicsSolver", "Repeating iteration " << t);
                continue;
            }
        }

        // backwards loop, duplicate of the forward one
        for (int t = T - 1; t > 0; -- t)
        {
            if (iteration == 0) Xhat_.col(t) = s_[t];
            else Xhat_.col(t) = (1 - alpha) * Xhat_.col(t) + alpha * b_[t];
            prob_->set_X(Xhat_);

            UpdateProcess(prob_->get_X(t), prob_->get_U(t), t);

            Eigen::MatrixXd Atinv = (A_[t] + Eigen::MatrixXd::Identity(A_[t].rows(), A_[t].cols()) * 1e-5).inverse();

            // TODO: Add white noise
            Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(NX, NX);
            if (t < T - 1) Q = prob_->get_F(t);

            Eigen::MatrixXd SplusRinv = (Sinv_[t - 1] + R_[t - 1] + Eigen::MatrixXd::Identity(R_[t - 1].rows(), R_[t - 1].cols()) * 1e-5).inverse();

            s_[t] = a_[t - 1] + A_[t - 1] * SplusRinv * (Sinv_[t - 1] * s_[t - 1] + r_[t - 1]);
            S_[t] = Q + B_[t - 1] * Hinv * B_[t - 1].transpose() + A_[t - 1] * SplusRinv * A_[t - 1].transpose();
            Sinv_[t] = (S_[t] + Eigen::MatrixXd::Identity(S_[t].rows(), S_[t].cols()) * 1e-5).inverse();

            if (t < T - 1)
            {
                Eigen::MatrixXd VplusRinv = (Vinv_[t + 1] + R_[t + 1] + Eigen::MatrixXd::Identity(R_[t + 1].rows(), R_[t + 1].cols()) * 1e-5).inverse();
                v_[t] = -Atinv * a_[t] + Atinv * VplusRinv * (Vinv_[t + 1] * v_[t + 1] + r_[t + 1]);
                V_[t] = -Atinv * (Q + B_[t] * Hinv * B_[t].transpose() + VplusRinv) * Atinv.transpose();
                Vinv_[t] = (V_[t] + Eigen::MatrixXd::Identity(V_[t].rows(), V_[t].cols()) * 1e-5).inverse();
            }

            Eigen::MatrixXd Binv = Sinv_[t] + Vinv_[t] + R_[t];
            AinvBSymPosDef(b_[t], Binv, Sinv_[t] * s_[t] + Vinv_[t] * v_[t] + r_[t]);

            if ((b_[t] - Xhat_.col(t)).norm() > theta)
            {
                t = t + 1; // +1 instead of -1
                // HIGHLIGHT_NAMED("AICODynamicsSolver", "Repeating iteration " << t);
                continue;
            }
        }

        // for (int t = 0; t < T - 1; ++t)
        //     HIGHLIGHT_NAMED("OI [" << t << "]", Xhat_.col(t));


        current_cost = ForwardPass();
        Uhat_ = prob_->get_U();

        for (int t = 0; t < T - 1; ++t)
        {
            solution.row(t) = Uhat_.col(t).transpose();
            prob_->Update(Uhat_.col(t), t);
        }

        prob_->SetCostEvolution(iteration, current_cost);

        // copy solutions for next iteration
        if (iteration == 0 || global_best_cost > current_cost)
        {
            global_best_cost = current_cost;
            last_best_iteration = iteration;
            best_U_ = Uhat_;
            best_X_ = Xhat_;
        }
            

        if (iteration - last_best_iteration > parameters_.FunctionTolerancePatience)
        {
            if (debug_) HIGHLIGHT_NAMED("AICODynamicsSolver", "Early stopping criterion reached. Time: " << planning_timer.GetDuration());
            break;
        }

        if (last_cost - current_cost < parameters_.FunctionTolerance && last_cost - current_cost > 0)
        {
            if (debug_) HIGHLIGHT_NAMED("AICODynamicsSolver", "Function tolerance reached. Time: " << planning_timer.GetDuration());
            break;
        }

        if (debug_ && iteration == GetNumberOfMaxIterations() - 1)
            HIGHLIGHT_NAMED("AICODynamicsSolver", "Max iterations reached. Time: " << planning_timer.GetDuration());

        if (debug_)
        {
            HIGHLIGHT_NAMED("AICODynamicsSolver", "Forward pass complete in " << planning_timer.GetDuration() << " with cost: " << current_cost);
            HIGHLIGHT_NAMED("AICODynamicsSolver", "Final state: " << prob_->get_X(T - 1).transpose());
        }

    }

    // for (int t = 0; t < T - 1; ++t)
    // {
    //     solution.row(t) = best_U_.col(t).transpose();
    //     prob_->Update(best_U_.col(t), t);
    // }
}

void AICODynamicsSolver::UpdateProcess(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, int t)
{
    const int T = prob_->get_T();
    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();
    const int NQ = prob_->get_num_positions();
    const double dt = dynamics_solver_->get_dt();
    // constexpr double alpha = 0.1;
    Eigen::MatrixXdRefConst B_control = dynamics_solver_->get_B();

    // Hurr-durr
    Eigen::MatrixXd H = prob_->get_R();
    Eigen::MatrixXd Hinv = (H + Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-5).inverse();

    // Eigen::VectorXd x = prob_->get_X(t);
    Eigen::VectorXd q = x.head(NQ);
    // TODO: get u from x
    // Eigen::VectorXd u = Eigen::VectorXd::Zero(NU);

    A_[t] = Eigen::MatrixXd::Identity(NX, NX);
    B_[t] = Eigen::MatrixXd::Zero(NX, NU);
    a_[t] = Eigen::VectorXd::Zero(NX);

    // Assuming pseudo dynamic process (M is identity matrix and F is zero vector)	
    Eigen::MatrixXd M = dynamics_solver_->get_M(x),
                    C = dynamics_solver_->get_C(x),
                    G = dynamics_solver_->get_G(x);
    Eigen::VectorXd xdot = dynamics_solver_->f(x, u);
    Eigen::MatrixXd F = M * xdot.tail(NQ) + G;

    Eigen::MatrixXd Minv = (Eigen::MatrixXd::Identity(M.rows(), M.cols()) * 1e-5 + M);

    // HIGHLIGHT_NAMED("AA", Minv.rows() << " x " << Minv.cols() << " * " << B_control.rows() << " x " << B_control.cols());
    Eigen::MatrixXd Minv_Bcontrol = Minv * B_control;
    // conditioning in case of underactuated robots
    Minv_Bcontrol = Minv_Bcontrol + Eigen::MatrixXd::Identity(Minv_Bcontrol.rows(), Minv_Bcontrol.cols()) * 1e-5;

    A_[t].topRightCorner(NQ, NQ).diagonal().setConstant(dt);

    B_[t].topLeftCorner(NQ, NU) = (Minv_Bcontrol * (dt * dt * 0.5));	
    B_[t].bottomLeftCorner(NQ, NU) = Minv_Bcontrol * dt;

    a_[t].head(NQ) = Minv * F * (dt * dt * 0.5);
    a_[t].tail(NQ) = Minv * F * dt;

    // R, r
    UpdateTaskCosts(t);
}

double AICODynamicsSolver::UpdateTaskCosts(int t)
{
    double C = 0;
    Eigen::MatrixXd Jt;
    double prec;
    rhat[t] = 0;

    // TODO: Is this correct?
    R_[t] = prob_->get_Q(t);
    r_[t].setZero();
    for (int i = 0; i < prob_->cost.num_tasks; ++i)
    {
        prec = prob_->cost.rho[t](i);
        if (prec > 0)
        {
            int start = prob_->cost.indexing[i].start_jacobian;
            int len = prob_->cost.indexing[i].length_jacobian;
            Jt = prob_->cost.jacobian[t].middleRows(start, len).transpose();
            C += prec * (prob_->cost.ydiff[t].segment(start, len)).squaredNorm();
            R_[t] += prec * Jt * prob_->cost.jacobian[t].middleRows(start, len);
            r_[t] += prec * Jt * (-prob_->cost.ydiff[t].segment(start, len) + prob_->cost.jacobian[t].middleRows(start, len) * qhat[t]);
            rhat[t] += prec * (-prob_->cost.ydiff[t].segment(start, len) + prob_->cost.jacobian[t].middleRows(start, len) * qhat[t]).squaredNorm();
        }
    }
    // return prob_->get_ct() * C;
    return C;
}

double AICODynamicsSolver::ForwardPass()
{
    const double Kp = parameters_.Kp_gain,
        Kd = parameters_.Kd_gain;

    double cost = 0;
    const int T = prob_->get_T();
    const Eigen::VectorXd control_limits = dynamics_solver_->get_control_limits();
    const double dt = dynamics_solver_->get_dt();

    const int NU = prob_->get_num_controls();
    const int NX = prob_->get_num_positions() + prob_->get_num_velocities();
    const int NQ = prob_->get_num_positions();

    // Pseudo-inverse of B
    Eigen::MatrixXd B = dynamics_solver_->get_B();
    Eigen::MatrixXd Binv = (B.transpose() * B).inverse() * B.transpose();

    for (int t = 0; t < T - 1; ++t)
    {
        Eigen::VectorXd xhat_diff = (Xhat_.col(t + 1) - Xhat_.col(t)) / dt,
            x = prob_->get_X(t);
        
        Eigen::VectorXd target_q = xhat_diff.tail(NQ)
            + Kp * (Xhat_.col(t).head(NQ) - x.head(NQ))
            + Kd * (Xhat_.col(t).tail(NQ) - x.tail(NQ));
        
        Eigen::VectorXd u = Binv * target_q;

        // clamp controls
        u = u.cwiseMax(-control_limits).cwiseMin(control_limits);

        prob_->Update(u, t);
        cost += dt * (prob_->GetControlCost(t) + prob_->GetStateCost(t));
    }

    // add terminal cost
    cost += prob_->GetStateCost(T - 1);
    return cost;
}

}  // namespace exotica
