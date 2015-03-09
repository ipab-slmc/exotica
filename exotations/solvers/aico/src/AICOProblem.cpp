/*
 * AICOProblem.cpp
 *
 *  Created on: 23 Apr 2014
 *      Author: Vladimir Ivan
 */

#include "aico/AICOProblem.h"

REGISTER_PROBLEM_TYPE("AICOProblem",exotica::AICOProblem);

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) {INDICATE_FAILURE; return PAR_ERR;}}
#define XML_OK(x) if(!ok(x)){INDICATE_FAILURE; return PAR_ERR;}

namespace exotica
{

	AICOProblem::AICOProblem(): T(0), tau(0), Q_rate(0), W_rate(0), H_rate(0)
	{
		// TODO Auto-generated constructor stub

	}

	AICOProblem::~AICOProblem ()
	{
		// TODO Auto-generated destructor stub
	}

	EReturn AICOProblem::initDerived(tinyxml2::XMLHandle & handle)
	{
        EReturn ret_value = SUCCESS;
		tinyxml2::XMLElement* xmltmp;
		bool hastime=false;
		XML_CHECK("T"); XML_OK(getInt(*xmltmp,T));
		if(T<=0) {INDICATE_FAILURE; return PAR_ERR;}
		xmltmp=handle.FirstChildElement("duration").ToElement();if (xmltmp) {XML_OK(getDouble(*xmltmp,tau));tau=tau/((double)T);hastime=true;}
		if(hastime)
		{
			xmltmp=handle.FirstChildElement("tau").ToElement();
			if (xmltmp) WARNING("Duration has already been specified, tau is ignored.");
		}
		else
		{
			XML_CHECK("tau"); XML_OK(getDouble(*xmltmp,tau));
		}

		XML_CHECK("Qrate"); XML_OK(getDouble(*xmltmp,Q_rate));
		XML_CHECK("Hrate"); XML_OK(getDouble(*xmltmp,H_rate));
		XML_CHECK("Wrate"); XML_OK(getDouble(*xmltmp,W_rate));
		{
			Eigen::VectorXd tmp;
			XML_CHECK("W"); XML_OK(getVector(*xmltmp,tmp));
			W=Eigen::MatrixXd::Identity(tmp.rows(),tmp.rows());
			W.diagonal() = tmp;
		}
        // Set number of time steps
        return setTime(T);
	}

	TaskDefinition_map& AICOProblem::getTaskDefinitions()
	{
		return task_defs_;
	}

    TaskMap_map& AICOProblem::getTaskMaps()
    {
        return task_maps_;
    }

	int AICOProblem::getT()
	{
		return T;
	}
        
    EReturn AICOProblem::setTime(int T_)
	{
        if(T_<=0) {INDICATE_FAILURE; return PAR_ERR;}
        tau=(double)T*tau/(double)T_;
        T=T_;
		// Set number of time steps
		EReturn ret_value = SUCCESS;
        for (TaskDefinition_map::const_iterator it = task_defs_.begin();
                it != task_defs_.end() and ok(ret_value); ++it)
		{
		    EReturn temp_return = it->second->setTimeSteps(T+1);
		    if (temp_return)
		    {
		        ret_value = temp_return;
		    }
		}
        return ret_value;
	}

	void AICOProblem::getT(int& T_)
	{
		T_=T;
	}

	double AICOProblem::getTau()
	{
		return tau;
	}

	void AICOProblem::getTau(double& tau_)
	{
		tau_=tau;
	}

	double AICOProblem::getDuration()
	{
		return tau*(double)T;
	}

	Eigen::MatrixXd AICOProblem::getW()
	{
		return W;
	}

	double AICOProblem::getQrate()
	{
		return Q_rate;
	}

	double AICOProblem::getWrate()
	{
		return W_rate;
	}

	double AICOProblem::getHrate()
	{
		return H_rate;
	}

} /* namespace exotica */
