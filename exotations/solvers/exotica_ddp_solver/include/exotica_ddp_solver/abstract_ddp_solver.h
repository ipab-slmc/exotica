//
// Copyright (c) 2019-2020, University of Edinburgh, University of Oxford
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

#ifndef EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_
#define EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_

#include <exotica_core/feedback_motion_solver.h>
#include <exotica_core/problems/dynamic_time_indexed_shooting_problem.h>
#include <exotica_core/server.h>
#include <exotica_core/tools/conversions.h>
#include <exotica_ddp_solver/abstract_ddp_solver_initializer.h>

namespace exotica
{
// \brief Base DDP Solver class that implements the forward pass.
//  and utility functions.
class AbstractDDPSolver : public FeedbackMotionSolver
{
public:
    ///\brief Solves the problem
    ///@param solution Returned solution trajectory as a vector of joint configurations.
    void Solve(Eigen::MatrixXd& solution) override;

    ///\brief Binds the solver to a specific problem which must be pre-initalised
    ///@param pointer Shared pointer to the motion planning problem
    ///@return        Successful if the problem is a valid DynamicTimeIndexedProblem
    void SpecifyProblem(PlanningProblemPtr pointer) override;

    Eigen::VectorXd GetFeedbackControl(Eigen::VectorXdRefConst x, int t) const override;

    const std::vector<Eigen::MatrixXd>& get_Vxx() const;
    const std::vector<Eigen::VectorXd>& get_Vx() const;
    const std::vector<Eigen::MatrixXd>& get_Qxx() const;
    const std::vector<Eigen::MatrixXd>& get_Qux() const;
    const std::vector<Eigen::MatrixXd>& get_Quu() const;
    const std::vector<Eigen::VectorXd>& get_Qx() const;
    const std::vector<Eigen::VectorXd>& get_Qu() const;
    const std::vector<Eigen::MatrixXd>& get_K() const;
    const std::vector<Eigen::VectorXd>& get_k() const;

    const std::vector<Eigen::VectorXd>& get_X_try() const;
    const std::vector<Eigen::VectorXd>& get_U_try() const;

    const std::vector<Eigen::VectorXd>& get_X_ref() const;
    const std::vector<Eigen::VectorXd>& get_U_ref() const;

    const std::vector<Eigen::MatrixXd>& get_Quu_inv() const;
    const std::vector<Eigen::MatrixXd>& get_fx() const;
    const std::vector<Eigen::MatrixXd>& get_fu() const;

    std::vector<double> get_control_cost_evolution() const;
    void set_control_cost_evolution(const int index, const double cost);

    std::vector<double> get_steplength_evolution() const;
    // void set_steplength_evolution(const int index, const double cost);

    std::vector<double> get_regularization_evolution() const;
    // void set_regularization_evolution(const int index, const double cost);

protected:
    DynamicTimeIndexedShootingProblemPtr prob_;  ///< Shared pointer to the planning problem.
    DynamicsSolverPtr dynamics_solver_;          ///< Shared pointer to the dynamics solver.

    ///\brief Computes the control gains for a the trajectory in the associated
    ///     DynamicTimeIndexedProblem.
    virtual void BackwardPass() = 0;

    ///\brief Forward simulates the dynamics using the gains computed in the
    ///     last BackwardPass;
    /// @param alpha The learning rate.
    /// @param ref_trajectory The reference state trajectory.
    /// @return The cost associated with the new control and state trajectory.
    double ForwardPass(const double alpha);

    AbstractDDPSolverInitializer base_parameters_;

    virtual void IncreaseRegularization()
    {
        lambda_ *= 10.;
    }

    virtual void DecreaseRegularization()
    {
        lambda_ /= 10.;
    }

    // Local variables used in the solver - copies get updated at the beginning of solve:
    Eigen::VectorXd alpha_space_;  ///< Backtracking line-search steplengths
    double lambda_;                ///< Regularization (Vxx, Quu)
    int T_;                        ///< Length of shooting problem, i.e., state trajectory. The control trajectory has length T_-1
    int NU_;                       ///< Size of control vector
    int NX_;                       ///< Size of state vector
    int NDX_;                      ///< Size of tangent vector to the state vector
    int NV_;                       ///< Size of velocity vector (tangent vector to the configuration)
    double dt_;                    ///< Integration time-step
    double cost_;                  ///< Cost during iteration
    double control_cost_;          ///< Control cost during iteration

    double cost_try_;          //!< Total cost computed by line-search procedure
    double control_cost_try_;  //!< Total control cost computed by line-search procedure

    double cost_prev_;   ///< Cost during previous iteration
    double alpha_best_;  ///< Line-search step taken
    double time_taken_forward_pass_, time_taken_backward_pass_;

    std::vector<Eigen::MatrixXd> Vxx_;  ///< Hessian of the Value function
    std::vector<Eigen::VectorXd> Vx_;   ///< Gradient of the Value function
    std::vector<Eigen::MatrixXd> Qxx_;  ///< Hessian of the Hamiltonian
    std::vector<Eigen::MatrixXd> Qux_;  ///< Hessian of the Hamiltonian
    std::vector<Eigen::MatrixXd> Quu_;  ///< Hessian of the Hamiltonian
    std::vector<Eigen::VectorXd> Qx_;   ///< Gradient of the Hamiltonian
    std::vector<Eigen::VectorXd> Qu_;   ///< Gradient of the Hamiltonian
    std::vector<Eigen::MatrixXd> K_;    ///< Feedback gains
    std::vector<Eigen::VectorXd> k_;    ///< Feed-forward terms

    std::vector<Eigen::VectorXd> X_try_;  ///< Updated state trajectory during iteration (computed by line-search)
    std::vector<Eigen::VectorXd> U_try_;  ///< Updated control trajectory during iteration (computed by line-search)

    std::vector<Eigen::VectorXd> X_ref_;  ///< Reference state trajectory for feedback control
    std::vector<Eigen::VectorXd> U_ref_;  ///< Reference control trajectory for feedback control

    std::vector<Eigen::MatrixXd> Quu_inv_;  ///< Inverse of the Hessian of the Hamiltonian
    std::vector<Eigen::MatrixXd> fx_;       ///< Derivative of the dynamics f w.r.t. x
    std::vector<Eigen::MatrixXd> fu_;       ///< Derivative of the dynamics f w.r.t. u

    std::vector<double> control_cost_evolution_;    ///< Evolution of the control cost (control regularization)
    std::vector<double> steplength_evolution_;      ///< Evolution of the steplength
    std::vector<double> regularization_evolution_;  ///< Evolution of the regularization (xreg/ureg)
};

}  // namespace exotica

#endif  // EXOTICA_DDP_SOLVER_ABSTRACT_DDP_SOLVER_H_
