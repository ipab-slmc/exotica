/*
 * ik_problem.cpp
 *
 *  Created on: 15 Jul 2014
 *      Author: yiming
 */

#include "ik_solver/ik_problem.h"

REGISTER_PROBLEM_TYPE("IKProblem", exotica::IKProblem);
#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}
namespace exotica
{
	IKProblem::IKProblem():tau_(0.01)
	{

	}

	IKProblem::~IKProblem()
	{
		//TODO
	}

	EReturn IKProblem::initDerived(tinyxml2::XMLHandle & handle)
	{
		tinyxml2::XMLElement* xmltmp;
		xmltmp = handle.FirstChildElement("W").ToElement();
		if (xmltmp)
		{
			Eigen::VectorXd tmp;
			XML_OK(getVector(*xmltmp, tmp));
			config_w_ = Eigen::MatrixXd::Identity(tmp.rows(), tmp.rows());
			config_w_.diagonal() = tmp;
		}
		xmltmp = handle.FirstChildElement("Tolerance").ToElement();
		if (xmltmp)
		{
			XML_OK(getDouble(*xmltmp, tau_));
		}
		return SUCCESS;
	}

	TaskDefinition_map& IKProblem::getTaskDefinitions()
	{
		return task_defs_;
	}

	TaskMap_map& IKProblem::getTaskMaps()
	{
		return task_maps_;
	}
	Eigen::MatrixXd IKProblem::getW()
	{
		return config_w_;
	}

	double IKProblem::getTau()
	{
		return tau_;
	}
}	//namespace exotica

