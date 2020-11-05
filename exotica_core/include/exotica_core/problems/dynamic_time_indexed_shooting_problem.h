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

#ifndef EXOTICA_CORE_DYNAMIC_TIME_INDEXED_SHOOTING_PROBLEM_H_
#define EXOTICA_CORE_DYNAMIC_TIME_INDEXED_SHOOTING_PROBLEM_H_

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/planning_problem.h>
#include <exotica_core/tasks.h>

#include <exotica_core/dynamic_time_indexed_shooting_problem_initializer.h>

namespace exotica
{
enum ControlCostLossTermType
{
    Undefined = -1,
    L2 = 0,
    SmoothL1 = 1,
    Huber = 2,
    PseudoHuber = 3,
};

class DynamicTimeIndexedShootingProblem : public PlanningProblem, public Instantiable<DynamicTimeIndexedShootingProblemInitializer>
{
public:
    DynamicTimeIndexedShootingProblem();
    virtual ~DynamicTimeIndexedShootingProblem();

    void Instantiate(const DynamicTimeIndexedShootingProblemInitializer& init) override;

    Eigen::VectorXd ApplyStartState(bool update_traj = true) override;
    void PreUpdate() override;
    void Update(Eigen::VectorXdRefConst u, int t);
    void Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, int t);
    void UpdateTerminalState(Eigen::VectorXdRefConst x);  // Updates the terminal state and recomputes the terminal cost - this is required e.g. when considering defects in the dynamics

    const int& get_T() const;     ///< Returns the number of timesteps in the state trajectory.
    void set_T(const int& T_in);  ///< Sets the number of timesteps in the state trajectory.

    const double& get_tau() const;  ///< Returns the discretization timestep tau

    const Eigen::MatrixXd& get_X() const;      ///< Returns the state trajectory X
    Eigen::VectorXd get_X(int t) const;        ///< Returns the state at time t
    void set_X(Eigen::MatrixXdRefConst X_in);  ///< Sets the state trajectory X (can be used as the initial guess)

    const Eigen::MatrixXd& get_U() const;      ///< Returns the control trajectory U
    Eigen::VectorXd get_U(int t) const;        ///< Returns the control state at time t
    void set_U(Eigen::MatrixXdRefConst U_in);  ///< Sets the control trajectory U (can be used as the initial guess)

    const Eigen::MatrixXd& get_X_star() const;           ///< Returns the target state trajectory X
    void set_X_star(Eigen::MatrixXdRefConst X_star_in);  ///< Sets the target state trajectory X

    const Eigen::MatrixXd& get_Q(int t) const;        ///< Returns the precision matrix at time step t
    void set_Q(Eigen::MatrixXdRefConst Q_in, int t);  ///< Sets the precision matrix for time step t

    const Eigen::MatrixXd& get_Qf() const;      ///< Returns the cost weight matrix at time N
    void set_Qf(Eigen::MatrixXdRefConst Q_in);  ///< Sets the cost weight matrix for time N

    const Eigen::MatrixXd& get_R() const;  ///< Returns the control weight matrix
    Eigen::MatrixXd get_F(int t) const;    ///< Returns the noise weight matrix at time t

    const Eigen::MatrixXd& GetControlNoiseJacobian(int column_idx) const;  ///< F[i]_u

    void EnableStochasticUpdates();
    void DisableStochasticUpdates();

    // TODO: Make private and add getter (no need to be public!)
    TimeIndexedTask cost;                  ///< Cost task
    std::vector<TaskSpaceVector> Phi;      ///< Stacked TaskMap vector
    std::vector<Eigen::MatrixXd> dPhi_dx;  ///< Stacked TaskMap Jacobian w.r.t. state
    std::vector<Eigen::MatrixXd> dPhi_du;  ///< Stacked TaskMap Jacobian w.r.t. control
    std::vector<Hessian> ddPhi_ddx;        ///< Stacked TaskMap Hessian w.r.t. state
    std::vector<Hessian> ddPhi_ddu;        ///< Stacked TaskMap Hessian w.r.t. control
    std::vector<Hessian> ddPhi_dxdu;       ///< Stacked TaskMap Hessian w.r.t. state and control

    // TODO: Make private and add getter/setter
    int length_Phi;       ///< Length of TaskSpaceVector (Phi => stacked task-maps)
    int length_jacobian;  ///< Length of tangent vector to Phi
    int num_tasks;        ///< Number of TaskMaps

    double GetStateCost(int t) const;
    double GetControlCost(int t) const;

    Eigen::VectorXd GetStateCostJacobian(int t);    ///< lx
    Eigen::VectorXd GetControlCostJacobian(int t);  ///< lu
    Eigen::MatrixXd GetStateCostHessian(int t);     ///< lxx
    Eigen::MatrixXd GetControlCostHessian(int t);   ///< luu
    Eigen::MatrixXd GetStateControlCostHessian()
    {
        // NOTE: For quadratic costs this is always 0
        //  thus we return a scalar of size 1 and value 0
        // This is the same as returning an (int)0 but for type safety
        //  we instantiate eigen vectors instead.
        return Eigen::MatrixXd::Zero(scene_->get_num_controls(), scene_->get_num_state_derivative());
    };  ///< lxu == lux

    void OnSolverIterationEnd()
    {
        if (parameters_.LossType == "AdaptiveSmoothL1")
        {
            // Adaptive SmoothL1 Rule
            //      from "RetinaMask: Learning to predict masks improves state-of-the-art single-shot detection for free"
            const double momentum = 0.9;

            Eigen::VectorXd new_smooth_l1_mean_ = Eigen::VectorXd::Zero(scene_->get_num_controls());
            Eigen::VectorXd new_smooth_l1_std_ = Eigen::VectorXd::Zero(scene_->get_num_controls());

            for (int t = 0; t < T_; ++t)
            {
                for (int ui = 0; ui < scene_->get_num_controls(); ++ui)
                {
                    new_smooth_l1_mean_[ui] += std::abs(U_.col(t)[ui]);
                }
            }

            new_smooth_l1_mean_ /= T_;

            for (int t = 0; t < T_; ++t)
            {
                for (int ui = 0; ui < scene_->get_num_controls(); ++ui)
                {
                    new_smooth_l1_std_[ui] += (std::abs(U_.col(t)[ui]) - new_smooth_l1_mean_[ui]) * (std::abs(U_.col(t)[ui]) - new_smooth_l1_mean_[ui]);
                }
            }

            new_smooth_l1_std_ /= T_;

            smooth_l1_mean_ = smooth_l1_mean_ * momentum + new_smooth_l1_mean_ * (1 - momentum);
            smooth_l1_std_ = smooth_l1_std_ * momentum + new_smooth_l1_std_ * (1 - momentum);

            for (int ui = 0; ui < scene_->get_num_controls(); ++ui)
            {
                l1_rate_[ui] = std::max(0.0, std::min(l1_rate_[ui], smooth_l1_mean_[ui] - smooth_l1_std_[ui]));
            }
        }
    }

    const ControlCostLossTermType& get_loss_type() const { return loss_type_; }
    void set_loss_type(const ControlCostLossTermType& loss_type_in) { loss_type_ = loss_type_in; }
    double get_control_cost_weight() const { return control_cost_weight_; }
    void set_control_cost_weight(const double control_cost_weight_in) { control_cost_weight_ = control_cost_weight_in; }
protected:
    /// \brief Checks the desired time index for bounds and supports -1 indexing.
    inline void ValidateTimeIndex(int& t_in) const
    {
        if (t_in >= T_ || t_in < -1)
        {
            ThrowPretty("Requested t=" << t_in << " out of range, needs to be 0 =< t < " << T_);
        }
        else if (t_in == -1)
        {
            t_in = (T_ - 1);
        }
    }
    void ReinitializeVariables();

    void UpdateTaskMaps(Eigen::VectorXdRefConst x, Eigen::VectorXdRefConst u, int t);

    int T_;       ///< Number of time steps
    double tau_;  ///< Time step duration
    bool stochastic_matrices_specified_ = false;
    bool stochastic_updates_enabled_ = false;

    Eigen::MatrixXd X_;       ///< State trajectory (i.e., positions, velocities). Size: num-states x T
    Eigen::MatrixXd U_;       ///< Control trajectory. Size: num-controls x (T-1)
    Eigen::MatrixXd X_star_;  ///< Goal state trajectory (i.e., positions, velocities). Size: num-states x T
    Eigen::MatrixXd X_diff_;  ///< Difference between X_ and X_star_. Size: ndx x T

    Eigen::MatrixXd Qf_;              ///< Final state cost
    std::vector<Eigen::MatrixXd> Q_;  ///< State space penalty matrix (precision matrix), per time index
    Eigen::MatrixXd R_;               ///< Control space penalty matrix

    std::vector<Eigen::MatrixXd> Ci_;  ///< Noise weight terms
    Eigen::MatrixXd CW_;               ///< White noise covariance

    // Pre-allocated variables
    std::vector<Eigen::MatrixXd> dxdiff_;
    std::vector<Eigen::VectorXd> state_cost_jacobian_;
    std::vector<Eigen::MatrixXd> state_cost_hessian_;
    std::vector<Eigen::VectorXd> general_cost_jacobian_;
    std::vector<Eigen::MatrixXd> general_cost_hessian_;
    std::vector<Eigen::VectorXd> control_cost_jacobian_;
    std::vector<Eigen::MatrixXd> control_cost_hessian_;

    std::vector<std::shared_ptr<KinematicResponse>> kinematic_solutions_;

    std::mt19937 generator_;
    std::normal_distribution<double> standard_normal_noise_{0, 1};

    TaskSpaceVector cost_Phi;

    double control_cost_weight_ = 1;
    ControlCostLossTermType loss_type_;
    void InstantiateCostTerms(const DynamicTimeIndexedShootingProblemInitializer& init);

    // sparsity costs
    Eigen::VectorXd l1_rate_;
    Eigen::VectorXd huber_rate_;
    Eigen::VectorXd bimodal_huber_mode1_, bimodal_huber_mode2_;

    Eigen::VectorXd smooth_l1_mean_, smooth_l1_std_;
};
typedef std::shared_ptr<exotica::DynamicTimeIndexedShootingProblem> DynamicTimeIndexedShootingProblemPtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_DYNAMIC_TIME_INDEXED_SHOOTING_PROBLEM_H_
