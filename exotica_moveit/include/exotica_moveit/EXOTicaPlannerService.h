/*
 * EXOTicaPlannerService.h
 *
 *  Created on: 19 Mar 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_
#define EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_

#include "aico/AICOsolver.h"
#include <ompl_solver/OMPLsolver.h>
#include "ik_solver/ik_solver.h"
#include <exotica/Initialiser.h>
#include "exotica_moveit/ExoticaPlanning.h"
#include <ros/package.h>

namespace exotica
{
	class EXOTicaPlannerService
	{
		public:
			EXOTicaPlannerService();
			~EXOTicaPlannerService();

			/*
			 * \brief	Solve function
			 * @param	q0			Start configuration
			 * @param	xml_file	Exotica initialisation file
			 * @param	solution	The solution
			 * @param	time		Planning time
			 */
			EReturn solve(const Eigen::VectorXd & q0, const std::string & xml_file,
					Eigen::MatrixXd & solution, double time);

		private:
			bool EXOTicaPlannning(exotica_moveit::ExoticaPlanning::Request & req,
					exotica_moveit::ExoticaPlanning::Response & res);
			ros::NodeHandle nh_;
			ros::AsyncSpinner sp_;
			ros::ServiceClient client_;
	};

	EReturn vectorExoticaToEigen(const exotica::Vector & exotica, Eigen::VectorXd & eigen)
	{
		eigen.resize(exotica.data.size());
		for (int i = 0; i < exotica.data.size(); i++)
			eigen(i) = exotica.data[i];
		return SUCCESS;
	}

	EReturn vectorEigenToExotica(const Eigen::VectorXd & eigen, exotica::Vector & exotica)
	{
		exotica.data.resize(eigen.rows());
		for (int i = 0; i < eigen.rows(); i++)
			exotica.data[i] = eigen(i);
		return SUCCESS;
	}

	EReturn matrixExoticaToEigen(const exotica::Matrix & exotica, Eigen::MatrixXd & eigen)
	{
		if (exotica.col * exotica.row != exotica.data.size())
		{
			ERROR("Matrix conversion failed, size mismatch."<<exotica.col<<" * "<<exotica.row<<" != "<<exotica.data.size());
			return FAILURE;
		}
		eigen.resize(exotica.row, exotica.col);
		int cnt = 0;
		for (int r = 0; r < exotica.row; r++)
			for (int c = 0; c < exotica.col; c++)
			{
				eigen(r, c) = exotica.data[cnt];
				cnt++;
			}
		return SUCCESS;
	}

	EReturn matrixEigenToExotica(const Eigen::MatrixXd & eigen, exotica::Matrix & exotica)
	{
		exotica.row = eigen.rows();
		exotica.col = eigen.cols();
		exotica.data.resize(exotica.col * exotica.row);
		int cnt = 0;
		for (int r = 0; r < exotica.row; r++)
			for (int c = 0; c < exotica.col; c++)
			{
				exotica.data[cnt] = eigen(r, c);
				cnt++;
			}
		return SUCCESS;
	}
}
;

#endif /* EXOTICA_EXOTICA_MOVEIT_INCLUDE_EXOTICA_MOVEIT_EXOTICAPLANNERSERVICE_H_ */
