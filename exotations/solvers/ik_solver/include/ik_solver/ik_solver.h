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
			 * \brief	Solves the problem
			 * @param	q0			Start state.
			 * @param	solution	This will be filled with the solution in joint space.
			 * @return	SUCESS if solution has been found, corresponding error code if not.
			 */
			EReturn Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution);
            EReturn Solve(Eigen::VectorXd q0, Eigen::MatrixXd & solution, int t);

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
			EReturn setGoal(const std::string & task_name, const Eigen::VectorXd & goal);

			/**
			 * \brief	Set rho
			 * @param	task_name	Task map name
			 * @param	rho	Rho
			 */
			EReturn setRho(const std::string & task_name, const double rho);

			IKProblem_ptr& getProblem();

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
            inline EReturn vel_solve(double & err, int t);
			IKProblem_ptr prob_; // Shared pointer to the planning problem.
			Eigen::VectorXd vel_vec_;	//Velocity vector
            Eigen::DiagonalMatrix<double, Eigen::Dynamic> task_weights; //!< Weight Matrices
			Eigen::MatrixXd big_jacobian, inv_jacobian; //!< Jacobian and its pseudo-Inverse
			Eigen::VectorXd task_error; //!< Task Error vector for the current optimisation level
			Eigen::VectorXd phi, goal;	// Goal and phi
            int maxdim_;
			int size_;	//Configuration size
			EParam<std_msgs::Int64> maxit_;	// Maximum iteration

			EParam<std_msgs::Float64> maxstep_;	// Maximum step
            std::vector<Eigen::VectorXd> rhos;
            Eigen::VectorXd diag;
            ros::Duration planning_time_;
	};
	typedef boost::shared_ptr<exotica::IKsolver> IKsolver_ptr;
}

#endif /* IK_SOLVER_H_ */
