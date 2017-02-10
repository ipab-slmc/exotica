/*
 *  Created on: 15 Jul 2014
 *      Author: Yiming Yang
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

#ifndef IK_SOLVER_H_
#define IK_SOLVER_H_

#include <exotica/EXOTica.hpp>
#include <exotica/Problems/IKProblem.h>
#include <exotica/Definitions/TaskSqrError.h>
#include <ik_solver/IKsolverInitializer.h>
#include <iostream>
#include <fstream>

namespace exotica
{
  /**
   * \brief	IK position solver
   */
  class IKsolver: public MotionSolver, public Instantiable<IKsolverInitializer>
  {
    public:
      IKsolver();
      virtual ~IKsolver();

      virtual void Instantiate(IKsolverInitializer& init);

      /**
       * \brief	Solves the problem. This returns the final state
       * @param	q0			Start state.
       * @param	solution	This will be filled with the solution in joint space(Vector).
       * @return	SUCESS if solution has been found, corresponding error code if not.
       */
      virtual void Solve(const Eigen::Ref<const Eigen::VectorXd> q0, Eigen::MatrixXd & solution);
      bool Solve(const Eigen::Ref<const Eigen::VectorXd> q0, Eigen::Ref<Eigen::MatrixXd> solution,
          int t);

      /**
       * \brief	Solves the problem. This returns the whole trajectory
       * @param	q0			Start state.
       * @param	solution	This will be filled with the solution in joint space(Vector).
       */
      bool SolveFullSolution(const Eigen::Ref<const Eigen::VectorXd> q0,
          Eigen::MatrixXd & solution);
      /**
       * \brief	Binds the solver to a specific problem which must be pre-initalised
       * @param	pointer	Shared pointer to the motion planning problem
       * @return	Successful if the problem is a valid AICOProblem
       */
      virtual void specifyProblem(PlanningProblem_ptr pointer);

      /*
       * \brief	Check if a problem is solvable by this solver (Pure Virtual)
       * @param	prob		Planning problem
       * @return	True if solvable, false otherwise
       */
      virtual bool isSolvable(const PlanningProblem_ptr & prob);

      /**
       * \brief	Set new goal
       * @param	task_name	Task map name
       * @param	goal	new goal
       */
      virtual void setGoal(const std::string & task_name,
          const Eigen::Ref<const Eigen::VectorXd> goal, int t = 0);
      virtual void getGoal(const std::string & task_name, Eigen::VectorXd& goal,int t = 0);

      /**
       * \brief	Set rho
       * @param	task_name	Task map name
       * @param	rho	Rho
       */
      void setRho(const std::string & task_name, const double rho,
          int t = 0);
      double getRho(const std::string & task_name, int t = 0);
      virtual void getRho(const std::string & task_name, double& rho, int t = 0);

      IKProblem_ptr& getProblem();

      double error;
      ros::Duration planning_time_;

      int getMaxIteration();
      int getLastIteration();
      void setReachGoal(const geometry_msgs::Pose &goal);
    protected:
      /**
       * \brief	Derived-elements initialiser: Pure Virtual
       * @param	handle	XMLHandle to the Solver element
       * @return	Should indicate success or otherwise
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

    private:
      /**
       * \brief	IK velocity solver
       * @param	err	Task error
       */

      IKsolverInitializer parameters_;

      inline void vel_solve(double & err, int t, const Eigen::Ref<const Eigen::VectorXd> q);
      IKProblem_ptr prob_; // Shared pointer to the planning problem.
      EParam<std_msgs::Int64> maxit_;	// Maximum iteration
      EParam<std_msgs::Float64> maxstep_;	// Maximum step
      EParam<std_msgs::Bool> multi_task_;
      std::map<std::string, std::pair<int, int> > taskIndex;

      std::vector<Eigen::VectorXd> rhos;
      std::vector<Eigen::MatrixXd> big_jacobian;
      std::vector<Eigen::VectorXd> goal;
      std::vector<Eigen::VectorXd> phi;
      std::vector<Eigen::VectorXi> dim;
      std::vector<Eigen::VectorXi> dimid;

      std::vector<std::vector<Eigen::Ref_ptr<Eigen::VectorXd>> > _rhos;
      std::vector<std::vector<Eigen::Ref_ptr<Eigen::MatrixXd>> > _jacobian;
      std::vector<std::vector<Eigen::Ref_ptr<Eigen::VectorXd>> > _goal;
      std::vector<std::vector<Eigen::Ref_ptr<Eigen::VectorXd>> > _phi;

      //Eigen::DiagonalMatrix<double, Eigen::Dynamic> Cinv; //!< Weight Matrices
      //Eigen::DiagonalMatrix<double, Eigen::Dynamic> C;
      //Eigen::DiagonalMatrix<double, Eigen::Dynamic> W;
      //Eigen::DiagonalMatrix<double, Eigen::Dynamic> Winv;
      Eigen::MatrixXd Cinv; //!< Weight Matrices
      Eigen::MatrixXd C;
      Eigen::MatrixXd W;
      Eigen::MatrixXd Winv;
      std::vector<Eigen::MatrixXd> weights;
      Eigen::VectorXd vel_vec_;	//Velocity vector
      Eigen::VectorXd task_error; //!< Task Error vector for the current optimisation level
      std::vector<Eigen::MatrixXd> JTCinv_;
      Eigen::MatrixXd JTCinvJ_;
      Eigen::VectorXd JTCinvdy_;
      TaskDefinition_map tasks_;
      int maxdim_;
      int size_;	//Configuration size
      Eigen::MatrixXd inv_jacobian;
      Eigen::VectorXd diag;

      Eigen::VectorXd qmin_,qmax_;

      int T;
      bool initialised_;
      int iterations_;

      ///	For FRRT debug
      bool FRRT_;

      ros::Publisher jac_pub_;
      visualization_msgs::MarkerArray jac_arr_;
      std::pair<int, int> coll_index_;
      std::pair<int, int> goal_index_;

      geometry_msgs::Pose reach_goal_;
  };
  typedef boost::shared_ptr<exotica::IKsolver> IKsolver_ptr;
}

#endif /* IK_SOLVER_H_ */
