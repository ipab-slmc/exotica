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
#include <iostream>
#include <fstream>

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
			EReturn Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution);
			EReturn Solve(Eigen::VectorXdRefConst q0, Eigen::MatrixXdRef solution, int t);

			/**
			 * \brief	Solves the problem. This returns the whole trajectory
			 * @param	q0			Start state.
			 * @param	solution	This will be filled with the solution in joint space(Vector).
			 * @return	SUCESS if solution has been found, corresponding error code if not.
			 */
			EReturn SolveFullSolution(Eigen::VectorXdRefConst q0, Eigen::MatrixXd & solution);
			/**
			 * \brief	Binds the solver to a specific problem which must be pre-initalised
			 * @param	pointer	Shared pointer to the motion planning problem
			 * @return	Successful if the problem is a valid AICOProblem
			 */
			virtual EReturn specifyProblem(PlanningProblem_ptr pointer);

			/**
			 * \brief	Set new goal
			 * @param	task_name	Task map name
			 * @param	goal	new goal
			 */
			EReturn setGoal(const std::string & task_name, Eigen::VectorXdRefConst goal, int t = 0);

			/**
			 * \brief	Set rho
			 * @param	task_name	Task map name
			 * @param	rho	Rho
			 */
			EReturn setRho(const std::string & task_name, const double rho, int t = 0);

			IKProblem_ptr& getProblem();

			double error;
			ros::Duration planning_time_;

			int getMaxIteration();
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
			std::map<std::string, std::pair<int, int> > taskIndex;

			std::vector<Eigen::VectorXd> rhos;
			std::vector<Eigen::MatrixXd> big_jacobian;
			std::vector<Eigen::VectorXd> goal;
			std::vector<Eigen::VectorXd> phi;
			std::vector<Eigen::VectorXi> dim;

			Eigen::DiagonalMatrix<double, Eigen::Dynamic> task_weights; //!< Weight Matrices
			Eigen::VectorXd vel_vec_;	//Velocity vector
			Eigen::VectorXd task_error; //!< Task Error vector for the current optimisation level
			int maxdim_;
			int size_;	//Configuration size
			Eigen::MatrixXd inv_jacobian;
			Eigen::VectorXd diag;

			int T;
			bool initialised_;
	};
	typedef boost::shared_ptr<exotica::IKsolver> IKsolver_ptr;
}

#endif /* IK_SOLVER_H_ */
