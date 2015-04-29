/*
 * EXOTicaPlannerService.cpp
 *
 *  Created on: 19 Mar 2015
 *      Author: yiming
 */

#include "exotica_moveit/EXOTicaPlannerService.h"

namespace exotica
{
	EReturn vectorExoticaToEigen(const exotica::Vector & exotica, Eigen::VectorXd & eigen)
	{
		eigen.resize(exotica.data.size());
		for (int i = 0; i < exotica.data.size(); i++)
			eigen(i) = exotica.data[i];
		return SUCCESS;
	}

	EReturn vectorEigenToExotica(Eigen::VectorXdRefConst eigen, exotica::Vector & exotica)
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
	EXOTicaPlannerService::EXOTicaPlannerService() :
			nh_("/ExoticaPlanningService"), sp_(2)
	{
		client_ = nh_.serviceClient<exotica_moveit::ExoticaPlanning>("exotica_planning");
		sp_.start();
	}

	EXOTicaPlannerService::~EXOTicaPlannerService()
	{

	}

	EReturn EXOTicaPlannerService::solve(Eigen::VectorXdRefConst q0, const std::string & xml_file,
			Eigen::MatrixXd & solution, double time)
	{
		exotica_moveit::ExoticaPlanning srv;
		vectorEigenToExotica(q0, srv.request.q0);
		srv.request.xml_file_ = xml_file;
		if (client_.call(srv))
		{
			if (srv.response.succeeded_)
			{
				if (ok(matrixExoticaToEigen(srv.response.solution_, solution)))
				{
					time = srv.response.planning_time_;
					return SUCCESS;
				}
				else
					INDICATE_FAILURE
			}
			INDICATE_FAILURE
		}
		INDICATE_FAILURE
		return FAILURE;

	}
}

