/*
 *  Created on: 19 Apr 2014
 *      Author: Vladimir Ivan
 *
 *  This code is based on algorithm developed by Marc Toussaint
 *  M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
 *  http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
 *  Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html
 *
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

/*! \mainpage
 * The \ref AICO is a solver within the EXOTica library.
 */

/**
 * \defgroup AICO Approximate Inference Control (AICO) solver
 * @{
 * The AICO solver was designed to solve finite time horizon time discretized (\f$T\f$ number of time steps) motion planning problem.
 * The AICO solver is defined within the EXOTica framework, therefore it makes use of a specification of the exotica planning problem class (\ref AICOProblem) and the underlying tools for initialisation and kinematic computations.
 * The inputs of the system are:
 *   - The start point \f$x_0\f$
 *   - Problem definition (\ref exotica::AICOProblem)
 * @}
 */

/**
 * \defgroup math Math functions
 * @{
 * This is a set of math function extending the functionality of Eigen library.
 * @}
 */

/** \file AICOsolver.h
 \brief Approximate Inference Control */

#ifndef AICOSOLVER_H_
#define AICOSOLVER_H_

#include <exotica/EXOTica.hpp>
#include <aico/AICOProblem.h>
#include <task_definition/TaskSqrError.h>
#include <iostream>
#include <fstream>
#include <aico/incremental_gaussian.h>

#include "lapack/cblas.h"
#include "f2c.h"
#undef small
#undef large
#include <lapack/clapack.h>
#undef double
#undef max
#undef min
#undef abs

namespace exotica
{
  /**
   * \brief Solves motion planning problem using Approximate Inference Control method.
   * \ingroup AICO
   */
  class AICOsolver: public MotionSolver
  {
    public:
      AICOsolver();
      virtual ~AICOsolver();
      /**
       * \brief Solves the problem
       * @param q0 Start state.
       * @param solution This will be filled with the solution in joint space.
       * @return SUCESS if solution has been found, corresponding error code if not.
       */
      void Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution);

      /**
       * \brief Solves the problem
       * @param q_init Trajectory initialisation.
       * @param solution This will be filled with the solution in joint space.
       * @return SUCESS if solution has been found, corresponding error code if not.
       */
      void Solve(const std::vector<Eigen::VectorXd>& q_init,
          Eigen::MatrixXd & solution);

      /**
       * \brief Binds the solver to a specific problem which must be pre-initalised
       * @param pointer Shared pointer to the motion planning problem
       * @return        Successful if the problem is a valid AICOProblem
       */
      virtual void specifyProblem(PlanningProblem_ptr pointer);

      /*
       * \brief	Check if a problem is solvable by this solver (Pure Virtual)
       * @param	prob		Planning problem
       * @return	True if solvable, false otherwise
       */
      virtual bool isSolvable(const PlanningProblem_ptr & prob);

      AICOProblem_ptr& getProblem();

      /**
       * \brief Stores costs into a file
       */
      void saveCosts(std::string file_name);

      /**
       * \brief Computes an inverse of symmetric positive definite matrix using LAPACK (very fast)
       * @param Ainv Resulting inverted matrix.
       * @param A A symmetric positive definite matrix to be inverted.
       */
      void inverseSymPosDef(Eigen::Ref<Eigen::MatrixXd> Ainv_,
          const Eigen::Ref<const Eigen::MatrixXd> & A_);

      /**
       * \brief Computes the solution to the linear problem \f$x=Ab\f$ for symmetric positive definite matrix A
       */
      void AinvBSymPosDef(Eigen::Ref<Eigen::VectorXd> x_,
          const Eigen::Ref<const Eigen::MatrixXd> & A_,
          const Eigen::Ref<const Eigen::VectorXd> & b_);

      void getStats(std::vector<SinglePassMeanCoviariance>& q_stat_);

      bool preupdateTrajectory_;

      /**
       * \brief	Set new goal
       * @param	task_name	Task map name
       * @param	goal	new goal
       * @param   t time step
       */
      void setGoal(const std::string & task_name,
          Eigen::VectorXdRefConst goal, int t = 0);

      /**
       * \brief	Set rho
       * @param	task_name	Task map name
       * @param	rho	Rho
       * @param   t time step
       */
      void setRho(const std::string & task_name, const double rho,
          int t = 0);

      /**
       * \brief	Get goal
       * @param	task_name	Task map name
       * @param	goal	returned goal
       * @param   t time step
       */
      void getGoal(const std::string & task_name, Eigen::VectorXd& goal,
          int t = 0);

      /**
       * \brief	Get rho
       * @param	task_name	Task map name
       * @param	goal	returned rho
       * @param   t time step
       */
      void getRho(const std::string & task_name, double& rho, int t = 0);

      std::vector<Eigen::VectorXd> y_star; //!< Task cost mappings
      std::vector<Eigen::VectorXd> rhos; //!< Task precisions
      std::map<std::string, std::pair<int, int> > taskIndex;
      Eigen::VectorXi dim; //!< Task dimension
      ros::Duration planning_time_;

    protected:
      /**
       * \brief Derived-elements initialiser: Pure Virtual
       * @param handle XMLHandle to the Solver element
       * @return       Should indicate success or otherwise
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

      /** \brief Initializes message data.
       *  @param q0 Start configuration
       *  @return  Indicates success
       */
      void initMessages();

      /**
       * \brief Initialise AICO messages from an initial trajectory
       * @param q_init Initial trajectory
       * @return  Indicates success
       */
      void initTrajectory(const std::vector<Eigen::VectorXd>& q_init);

    private:
      AICOProblem_ptr prob_; //!< Shared pointer to the planning problem.
      double damping; //!< Damping
      double damping_init; //!< Damping
      double tolerance; //!< Termination error threshold
      int max_iterations; //!< Max. number of AICO iterations
      bool useBwdMsg; //!< Flag for using backward message initialisation
      Eigen::VectorXd bwdMsg_v; //!< Backward message initialisation mean
      Eigen::MatrixXd bwdMsg_Vinv; //!< Backward message initialisation covariance
      bool dynamic; //!< Plan
      std::vector<Eigen::VectorXd> phiBar; //!< Task cost mappings
      std::vector<Eigen::MatrixXd> JBar; //!< Task cost Jacobians

      std::vector<SinglePassMeanCoviariance> q_stat; //!< Cost weigthed normal distribution of configurations across sweeps.

      std::vector<Eigen::VectorXd> s; //!< Forward message mean
      std::vector<Eigen::MatrixXd> Sinv; //!< Forward message covariance inverse
      std::vector<Eigen::VectorXd> v; //!< Backward message mean
      std::vector<Eigen::MatrixXd> Vinv; //!< Backward message covariance inverse
      std::vector<Eigen::VectorXd> r; //!< Task message mean
      std::vector<Eigen::MatrixXd> R; //!< Task message covariance
      Eigen::VectorXd rhat; //!< Task message point of linearisation
      std::vector<Eigen::VectorXd> b; //!< Belief mean
      std::vector<Eigen::MatrixXd> Binv; //!< Belief covariance inverse
      std::vector<Eigen::VectorXd> q; //!< Configuration space trajectory
      std::vector<Eigen::VectorXd> qhat; //!< Point of linearisation
      Eigen::VectorXd costControl; //!< Control cost for each time step
      Eigen::MatrixXd costTask; //!< Task cost for each task for each time step
      std::vector<std::string> taskNames; //!< Task names (only used for printing debug info)

      std::vector<Eigen::VectorXd> phiBar_old; //!< Task cost mappings (last most optimal value)
      std::vector<Eigen::MatrixXd> JBar_old; //!< Task cost Jacobians (last most optimal value)
      std::vector<Eigen::VectorXd> y_star_old; //!< Task cost mappings (last most optimal value)
      std::vector<Eigen::VectorXd> rhos_old; //!< Task precisions (last most optimal value)
      Eigen::VectorXi dim_old; //!< Task dimension

      std::vector<Eigen::VectorXd> s_old; //!< Forward message mean (last most optimal value)
      std::vector<Eigen::MatrixXd> Sinv_old; //!< Forward message covariance inverse (last most optimal value)
      std::vector<Eigen::VectorXd> v_old; //!< Backward message mean (last most optimal value)
      std::vector<Eigen::MatrixXd> Vinv_old; //!< Backward message covariance inverse (last most optimal value)
      std::vector<Eigen::VectorXd> r_old; //!< Task message mean (last most optimal value)
      std::vector<Eigen::MatrixXd> R_old; //!< Task message covariance (last most optimal value)
      Eigen::VectorXd rhat_old; //!< Task message point of linearisation (last most optimal value)
      std::vector<Eigen::VectorXd> b_old; //!< Belief mean (last most optimal value)
      std::vector<Eigen::MatrixXd> Binv_old; //!< Belief covariance inverse (last most optimal value)
      std::vector<Eigen::VectorXd> q_old; //!< Configuration space trajectory (last most optimal value)
      std::vector<Eigen::VectorXd> qhat_old; //!< Point of linearisation (last most optimal value)
      Eigen::VectorXd costControl_old; //!< Control cost for each time step (last most optimal value)
      Eigen::MatrixXd costTask_old; //!< Task cost for each task for each time step (last most optimal value)

      std::vector<Eigen::VectorXd> dampingReference; //!< Damping reference point
      double cost; //!< cost of MAP trajectory
      double cost_old; //!< cost of MAP trajectory (last most optimal value)
      double b_step; //!< Squared configuration space step

      std::vector<Eigen::MatrixXd> A; //!< State transition matrix
      std::vector<Eigen::MatrixXd> tA; //!< State transition matrix transpose
      std::vector<Eigen::MatrixXd> Ainv; //!< State transition matrix inverse
      std::vector<Eigen::MatrixXd> invtA; //!< State transition matrix transpose inverse
      std::vector<Eigen::VectorXd> a; //!< State transition drift
      std::vector<Eigen::MatrixXd> B; //!< Control matrix
      std::vector<Eigen::MatrixXd> tB; //!< Control matrix transpose
      std::vector<Eigen::MatrixXd> W; //!< Configuration space weight matrix inverse
      std::vector<Eigen::MatrixXd> H; //!< Integrated state transition covariance inverse
      std::vector<Eigen::MatrixXd> Winv; //!< Configuration space weight matrix inverse
      std::vector<Eigen::MatrixXd> Hinv; //!< Integrated state transition covariance inverse
      std::vector<Eigen::MatrixXd> Q; //!< State transition covariance

      int sweep; //!< Sweeps so far
      enum SweepMode
      {
        smForwardly = 0,
        smSymmetric,
        smLocalGaussNewton,
        smLocalGaussNewtonDamped
      };
      int sweepMode; //!< Sweep mode
      int T; //!< Number of time steps
      int n; //!< Configuration space size
      int updateCount;

      Eigen::MatrixXd linSolverTmp;

      /**
       * \brief Updates the forward message at time step $t$
       * @param t Time step
       *
       * Updates the mean and covariance of the forward message using:
       * \f$ \mu_{x_{t-1}\rightarrow x_t}(x)=\mathcal{N}(x_t|s_t,S_t) \f$
       * , where
       * \f$ s_t=a_{t-1}\!+\!A_{t-1}(S_{t-1}^{-1}\!+\!R_{t-1})^{-1}(S_{t-1}^{-1}s_{t-1}\!+\!r_{t-1}) \f$
       * and
       * \f$ S_t=Q+B_tH^{-1}B_t^{\!\top\!} + A_{t-1}(S_{t-1}^{-1}+R_{t-1})^{-1}A_{t-1}^{\!\top\!} \f$.
       */
      void updateFwdMessage(int t);
      /**
       * \brief Updates the backward message at time step $t$
       * @param t Time step
       *
       * Updates the mean and covariance of the backward message using:
       * \f$ \mu_{x_{t+1}\rightarrow x_t}(x)=\mathcal{N}(x_t|v_t,V_t) \f$
       * , where
       * \f$ v_t=-A_{t}^{-1}a_{t}\!\!+\!\!A_{t}^{-1}(V_{t+1}^{-1}\!\!+\!\!R_{t+1})^{-1}(V_{t+1}^{-1}v_{t+1}\!\!+\!\!r_{t+1}) \f$
       * and
       * \f$ V_t=A_{t}^{-1}[Q+B_tH^{-1}B_t^{\!\top\!} + (V_{t+1}^{-1}+R_{t+1})^{-1}]A_{t}^{-{\!\top\!}} \f$.
       */
      void updateBwdMessage(int t);
      /**
       * \brief Updates the task message at time step $t$
       * @param t Time step
       * @param qhat_t Point of linearisation at time step $t$
       * @param tolerance_ Lazy update tolerance (only update the task message if the state changed more than this value)
       * @param maxStepSize If step size >0, cap the motion at this step to the step size.
       *
       * Updates the mean and covariance of the task message using:
       * \f$ \mu_{z_t\rightarrow x_t}(x)=\mathcal{N}[x_t|r_t,R_t] \f$
       */
      void updateTaskMessage(int t,
          const Eigen::Ref<const Eigen::VectorXd> & qhat_t, double tolerance_,
          double maxStepSize = -1.);
      /**
       * \brief Update messages for given time step
       * @param t Time step.
       * @param updateFwd Update the forward message.
       * @param updateBwd Update the backward message.
       * @param maxRelocationIterations Maximum number of relocation while searching for a good linearisation point
       * @param tolerance_ Tolerance for for stopping the search.
       * @param forceRelocation Set to true to force relocation even when the result is within tolerance.
       * @param maxStepSize Step size for updateTaskMessage.
       */
      void updateTimeStep(int t, bool updateFwd, bool updateBwd,
          int maxRelocationIterations, double tolerance_, bool forceRelocation,
          double maxStepSize = -1.);
      /**
       * \brief Update messages for given time step using the Gauss Newton method
       * @param t Time step.
       * @param updateFwd Update the forward message.
       * @param updateBwd Update the backward message.
       * @param maxRelocationIterations Maximum number of relocation while searching for a good linearisation point
       * @param tolerance Tolerance for for stopping the search.
       * @param maxStepSize Step size for updateTaskMessage.
       *
       * First, the messages \f$ \mu_{x_{t-1}\rightarrow x_t}(x)=\mathcal{N}(x_t|s_t,S_t) \f$,
       * \f$ \mu_{x_{t+1}\rightarrow x_t}(x)=\mathcal{N}(x_t|v_t,V_t) \f$ and
       * \f$ \mu_{z_t\rightarrow x_t}(x)=\mathcal{N}[x_t|r_t,R_t] \f$
       * are computed. Then, the belief is updated:
       * \f$ b_t(X_t)=\mu_{x_{t-1}\rightarrow x_t}(x) \mu_{x_{t+1}\rightarrow x_t}(x) \mu_{z_t\rightarrow x_t}(x) \f$
       * where the mean and covariance are updated as follows:
       * \f$ b_t(X_t)=\mathcal{N}\left(x_t|(S_t^{-1}+V_t^{-1}+R_t)^{-1}(S_t^{-1}s_t+V_t^{-1}v_t+r_t),S_t^{-1}+V_t^{-1}+R_t \right) \f$.
       */
      void updateTimeStepGaussNewton(int t, bool updateFwd, bool updateBwd,
          int maxRelocationIterations, double tolerance, double maxStepSize =
              -1.);
      /**
       * \brief Computes the cost of the trajectory
       * @param x Trajecotry.
       * @return Cost of the trajectory.
       */
      double evaluateTrajectory(const std::vector<Eigen::VectorXd>& x,
          bool skipUpdate = false);
      /**
       * \brief Stores the previous state.
       */
      void rememberOldState();
      /**
       * \brief Reverts back to previous state if the cost of the current state is higher.
       */
      void perhapsUndoStep();

      /**
       * \brief Returns process transition matrices
       * @param A_ System transition matrix.
       * @param a_ System transition drift vector.
       * @param B_ System control matrix.
       */
      void getProcess(Eigen::Ref<Eigen::MatrixXd> A_,
          Eigen::Ref<Eigen::VectorXd> a_, Eigen::Ref<Eigen::MatrixXd> B_);

      /**
       * \brief Updates the task cost terms \f$ R, r, \hat{r} \f$ for time step \f$t\f$. AICOProblem::update() has to be called before calling this function.
       * @param t Time step to be updated.
       */
      double getTaskCosts(int t);

      /**
       * \brief Compute one step of the AICO algorithm.
       * @return Change in cost of the trajectory.
       */
      double step();
  };

  typedef boost::shared_ptr<exotica::AICOsolver> AICOsolver_ptr;
} /* namespace exotica */

#endif /* AICOSOLVER_H_ */
