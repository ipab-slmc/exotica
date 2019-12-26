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

#ifndef EXOTICA_AICO_SOLVER_BAYESIAN_IK_SOLVER_H_
#define EXOTICA_AICO_SOLVER_BAYESIAN_IK_SOLVER_H_

#include <iostream>

#include <exotica_core/motion_solver.h>
#include <exotica_core/problems/unconstrained_end_pose_problem.h>

#include <exotica_aico_solver/incremental_gaussian.h>
#include <exotica_aico_solver/math_operations.h>

#include <exotica_aico_solver/bayesian_ik_solver_initializer.h>

namespace exotica
{
/// \brief Solves motion planning problem using Approximate Inference Control method.
/// \ingroup AICO
class BayesianIKSolver : public MotionSolver, public Instantiable<BayesianIKSolverInitializer>
{
public:
    void Instantiate(const BayesianIKSolverInitializer& init) override;

    /// \brief Solves the problem
    /// @param solution Returned end pose solution.
    void Solve(Eigen::MatrixXd& solution);

    /// \brief Binds the solver to a specific problem which must be pre-initalised
    /// @param pointer Shared pointer to the motion planning problem
    /// @return        Successful if the problem is a valid UnconstrainedEndPoseProblem
    void SpecifyProblem(PlanningProblemPtr pointer) override;

protected:
    /// \brief Initializes message data.
    /// @param q0 Start configuration
    /// @return  Indicates success
    void InitMessages();

    /// \brief Initialise AICO messages from an initial trajectory
    /// @param q_init Initial trajectory
    /// @return  Indicates success
    void InitTrajectory(const Eigen::VectorXd& q_init);

private:
    UnconstrainedEndPoseProblemPtr prob_;   //!< Shared pointer to the planning problem.
    double damping = 0.01;                  //!< Damping
    double damping_init_ = 100.0;           //!< Damping
    double minimum_step_tolerance_ = 1e-5;  //!< Update tolerance to stop update of messages if change of maximum coefficient is less than this tolerance.
    double step_tolerance_ = 1e-5;          //!< Relative step tolerance (termination criterion)
    double function_tolerance_ = 1e-5;      //!< Relative function tolerance/first-order optimality criterion
    int max_backtrack_iterations_ = 10;     //!< Max. number of sweeps without improvement before terminating (= line-search)
    bool use_bwd_msg_ = false;              //!< Flag for using backward message initialisation
    Eigen::VectorXd bwd_msg_v_;             //!< Backward message initialisation mean
    Eigen::MatrixXd bwd_msg_Vinv_;          //!< Backward message initialisation covariance
    bool sweep_improved_cost_;              //!< Whether the last sweep improved the cost (for backtrack iterations count)
    int iteration_count_;                   //!< Iteration counter

    Eigen::VectorXd s;     //!< Forward message mean
    Eigen::MatrixXd Sinv;  //!< Forward message covariance inverse
    Eigen::VectorXd v;     //!< Backward message mean
    Eigen::MatrixXd Vinv;  //!< Backward message covariance inverse
    Eigen::VectorXd r;     //!< Task message mean
    Eigen::MatrixXd R;     //!< Task message covariance
    double rhat;           //!< Task message point of linearisation
    Eigen::VectorXd b;     //!< Belief mean
    Eigen::MatrixXd Binv;  //!< Belief covariance inverse
    Eigen::VectorXd q;     //!< Configuration space trajectory
    Eigen::VectorXd qhat;  //!< Point of linearisation

    Eigen::VectorXd s_old;     //!< Forward message mean (last most optimal value)
    Eigen::MatrixXd Sinv_old;  //!< Forward message covariance inverse (last most optimal value)
    Eigen::VectorXd v_old;     //!< Backward message mean (last most optimal value)
    Eigen::MatrixXd Vinv_old;  //!< Backward message covariance inverse (last most optimal value)
    Eigen::VectorXd r_old;     //!< Task message mean (last most optimal value)
    Eigen::MatrixXd R_old;     //!< Task message covariance (last most optimal value)
    double rhat_old;           //!< Task message point of linearisation (last most optimal value)
    Eigen::VectorXd b_old;     //!< Belief mean (last most optimal value)
    Eigen::MatrixXd Binv_old;  //!< Belief covariance inverse (last most optimal value)
    Eigen::VectorXd q_old;     //!< Configuration space trajectory (last most optimal value)
    Eigen::VectorXd qhat_old;  //!< Point of linearisation (last most optimal value)

    Eigen::VectorXd damping_reference_;                      //!< Damping reference point
    double cost_ = 0.0;                                      //!< cost of MAP trajectory
    double cost_old_ = std::numeric_limits<double>::max();   //!< cost of MAP trajectory (last most optimal value)
    double cost_prev_ = std::numeric_limits<double>::max();  //!< previous iteration cost
    double b_step_ = 0.0;                                    //!< Squared configuration space step
    double b_step_old_;

    Eigen::MatrixXd W;     //!< Configuration space weight matrix inverse
    Eigen::MatrixXd Winv;  //!< Configuration space weight matrix inverse

    int sweep_ = 0;  //!< Sweeps so far
    int best_sweep_ = 0;
    int best_sweep_old_ = 0;
    enum SweepMode
    {
        FORWARD = 0,
        SYMMETRIC,
        LOCAL_GAUSS_NEWTON,
        LOCAL_GAUSS_NEWTON_DAMPED
    };
    int sweep_mode_ = 0;  //!< Sweep mode
    int update_count_ = 0;

    bool verbose_ = false;

    /// \brief Updates the forward message
    /// Updates the mean and covariance of the forward message using:
    /// \f$ \mu_{x_{t-1}\rightarrow x_t}(x)=\mathcal{N}(x_t|s_t,S_t) \f$
    /// , where
    /// \f$ s_t=a_{t-1}\!+\!A_{t-1}(S_{t-1}^{-1}\!+\!R_{t-1})^{-1}(S_{t-1}^{-1}s_{t-1}\!+\!r_{t-1}) \f$
    /// and
    /// \f$ S_t=Q+B_tH^{-1}B_t^{\!\top\!} + A_{t-1}(S_{t-1}^{-1}+R_{t-1})^{-1}A_{t-1}^{\!\top\!} \f$.
    void UpdateFwdMessage();

    /// \brief Updates the backward message
    /// Updates the mean and covariance of the backward message using:
    /// \f$ \mu_{x_{t+1}\rightarrow x_t}(x)=\mathcal{N}(x_t|v_t,V_t) \f$
    /// , where
    /// \f$ v_t=-A_{t}^{-1}a_{t}\!\!+\!\!A_{t}^{-1}(V_{t+1}^{-1}\!\!+\!\!R_{t+1})^{-1}(V_{t+1}^{-1}v_{t+1}\!\!+\!\!r_{t+1}) \f$
    /// and
    /// \f$ V_t=A_{t}^{-1}[Q+B_tH^{-1}B_t^{\!\top\!} + (V_{t+1}^{-1}+R_{t+1})^{-1}]A_{t}^{-{\!\top\!}} \f$.
    void UpdateBwdMessage();

    /// \brief Updates the task message
    /// @param qhat_t Point of linearisation at time step $t$
    /// @param tolerance Lazy update tolerance (only update the task message if the state changed more than this value)
    /// @param max_step_size If step size >0, cap the motion at this step to the step size.
    /// Updates the mean and covariance of the task message using:
    /// \f$ \mu_{z_t\rightarrow x_t}(x)=\mathcal{N}[x_t|r_t,R_t] \f$
    void UpdateTaskMessage(const Eigen::Ref<const Eigen::VectorXd>& qhat_t, double tolerance,
                           double max_step_size = -1.);

    /// \brief Update messages for given time step
    /// @param t Time step.
    /// @param update_fwd Update the forward message.
    /// @param update_bwd Update the backward message.
    /// @param max_relocation_iterations Maximum number of relocation while searching for a good linearisation point
    /// @param tolerance Tolerance for for stopping the search.
    /// @param force_relocation Set to true to force relocation even when the result is within tolerance.
    /// @param max_step_size Step size for UpdateTaskMessage.
    void UpdateTimestep(bool update_fwd, bool update_bwd,
                        int max_relocation_iterations, double tolerance, bool force_relocation,
                        double max_step_size = -1.);

    /// \brief Update messages for given time step using the Gauss Newton method
    /// @param t Time step.
    /// @param update_fwd Update the forward message.
    /// @param update_bwd Update the backward message.
    /// @param max_relocation_iterations Maximum number of relocation while searching for a good linearisation point
    /// @param tolerance Tolerance for for stopping the search.
    /// @param max_step_size Step size for UpdateTaskMessage.
    /// First, the messages \f$ \mu_{x_{t-1}\rightarrow x_t}(x)=\mathcal{N}(x_t|s_t,S_t) \f$,
    /// \f$ \mu_{x_{t+1}\rightarrow x_t}(x)=\mathcal{N}(x_t|v_t,V_t) \f$ and
    /// \f$ \mu_{z_t\rightarrow x_t}(x)=\mathcal{N}[x_t|r_t,R_t] \f$
    /// are computed. Then, the belief is updated:
    /// \f$ b_t(X_t)=\mu_{x_{t-1}\rightarrow x_t}(x) \mu_{x_{t+1}\rightarrow x_t}(x) \mu_{z_t\rightarrow x_t}(x) \f$
    /// where the mean and covariance are updated as follows:
    /// \f$ b_t(X_t)=\mathcal{N}\left(x_t|(S_t^{-1}+V_t^{-1}+R_t)^{-1}(S_t^{-1}s_t+V_t^{-1}v_t+r_t),S_t^{-1}+V_t^{-1}+R_t \right) \f$.
    void UpdateTimestepGaussNewton(bool update_fwd, bool update_bwd,
                                   int max_relocation_iterations, double tolerance, double max_step_size = -1.);
    /// \brief Computes the cost of the trajectory
    /// @param x Trajecotry.
    /// @return Cost of the trajectory.
    double EvaluateTrajectory(const Eigen::VectorXd& x, bool skip_update = false);

    /// \brief Stores the previous state.
    void RememberOldState();

    /// \brief Reverts back to previous state if the cost of the current state is higher.
    void PerhapsUndoStep();

    /// \brief Updates the task cost terms \f$ R, r, \hat{r} \f$. UnconstrainedEndPoseProblem::Update() has to be called before calling this function.
    void GetTaskCosts();

    /// \brief Compute one step of the AICO algorithm.
    /// @return Change in cost of the trajectory.
    double Step();
};
}  // namespace exotica

#endif  // EXOTICA_AICO_SOLVER_BAYESIAN_IK_SOLVER_H_
