/*
 * ik_solver.h
 *
 *  Created on: 15 Jul 2014
 *      Author: yiming
 */

#ifndef IK_SOLVER_H_
#define IK_SOLVER_H_

#include <exotica/EXOTica.hpp>
#include <ik_solver/ik_problem.h>
#include <task_definition/TaskSqrError.h>
#include <generic/Identity.h>
#include <iostream>
#include <fstream>
#include <dynamic_reachability_map/DRMAction.h>
#include <actionlib/client/simple_action_client.h>

namespace exotica
{
  /**
   * \brief	IK position solver
   */
  class IKsolver: public MotionSolver
  {
    public:
      IKsolver();
      virtual ~IKsolver();

      /**
       * \brief	Solves the problem. This returns the final state
       * @param	q0			Start state.
       * @param	solution	This will be filled with the solution in joint space(Vector).
       * @return	SUCESS if solution has been found, corresponding error code if not.
       */
      virtual EReturn Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution);
      EReturn Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXdRef solution,
          int t);

      /**
       * \brief	Solves the problem. This returns the whole trajectory
       * @param	q0			Start state.
       * @param	solution	This will be filled with the solution in joint space(Vector).
       */
      EReturn SolveFullSolution(Eigen::VectorXdRefConst q0,
          Eigen::MatrixXd & solution);
      /**
       * \brief	Binds the solver to a specific problem which must be pre-initalised
       * @param	pointer	Shared pointer to the motion planning problem
       * @return	Successful if the problem is a valid AICOProblem
       */
      virtual EReturn specifyProblem(PlanningProblem_ptr pointer);

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
      EReturn setGoal(const std::string & task_name,
          Eigen::VectorXdRefConst goal, int t = 0);

      /**
       * \brief	Set rho
       * @param	task_name	Task map name
       * @param	rho	Rho
       */
      EReturn setRho(const std::string & task_name, const double rho,
          int t = 0);
      double getRho(const std::string & task_name, int t = 0);

      IKProblem_ptr& getProblem();

      double error;
      ros::Duration planning_time_;

      int getMaxIteration();
      Eigen::VectorXd nullSpaceRef;
      void setReachGoal(const geometry_msgs::Pose &goal);
    protected:
      /**
       * \brief	Derived-elements initialiser: Pure Virtual
       * @param	handle	XMLHandle to the Solver element
       * @return	Should indicate success or otherwise
       */
      virtual EReturn initDerived(tinyxml2::XMLHandle & handle);

    private:
      /**
       * \brief	IK velocity solver
       * @param	err	Task error
       */
      inline EReturn vel_solve(double & err, int t, Eigen::VectorXdRefConst q);
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

      std::vector<std::vector<Eigen::VectorXdRef_ptr> > _rhos;
      std::vector<std::vector<Eigen::MatrixXdRef_ptr> > _jacobian;
      std::vector<std::vector<Eigen::VectorXdRef_ptr> > _goal;
      std::vector<std::vector<Eigen::VectorXdRef_ptr> > _phi;

      Eigen::DiagonalMatrix<double, Eigen::Dynamic> task_weights; //!< Weight Matrices
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

      int T;
      bool initialised_;

      ///	For FRRT debug
      bool FRRT_;
      EParam<std_msgs::Float64> local_minima_threshold_;
      ros::Publisher jac_pub_;
      visualization_msgs::MarkerArray jac_arr_;
      std::pair<int, int> coll_index_;
      std::pair<int, int> goal_index_;
      EParam<std_msgs::Bool> ignore_obs_near_goal_;

      EParam<std_msgs::Bool> use_drm_;
      actionlib::SimpleActionClient<dynamic_reachability_map::DRMAction> drm_client_;
      geometry_msgs::Pose reach_goal_;
      Distance_Ptr reach_position_taskmap_;
      Orientation_Ptr reach_orientation_taskmap_;
  };
  typedef boost::shared_ptr<exotica::IKsolver> IKsolver_ptr;
}

#endif /* IK_SOLVER_H_ */
