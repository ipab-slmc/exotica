#include "exotica/TaskMap.h"

exotica::TaskMap::TaskMap()
{
	//!< Just reset the flags
	phi_ok_ = false;
	jac_ok_ = false;
}

exotica::EReturn exotica::TaskMap::initBase(tinyxml2::XMLHandle & handle, Server_ptr & server,
		const kinematica::KinematicScene_map & scene_ptr)
{
	//!< Clear flags and kinematic scene pointer
    Object::initBase(handle,server);
	invalidate();
	if (!server)
	{
		INDICATE_FAILURE
		return FAILURE;
	}
	server_ = server;
	scene_ = kinematica::KinematicScene_ptr();  //!< Null pointer

	if (handle.FirstChildElement("kscene").ToElement())
	{
		LOCK(scene_lock_);  //!< Local lock
		const char * name = handle.FirstChildElement("kscene").ToElement()->Attribute("name");
		if (name == nullptr)
		{
			INDICATE_FAILURE
			return PAR_ERR;
		}
		auto it = scene_ptr.find(name);
		if (it == scene_ptr.end())
		{
			INDICATE_FAILURE
			return PAR_ERR;
		}
		scene_ = it->second;
	}
	else
	{
		ERROR("No scene was specified!");
	}
	return initDerived(handle); //!< Call the derived member
}

exotica::EReturn exotica::TaskMap::phi(Eigen::Ref<Eigen::VectorXd> y, int t)
{
	LOCK(phi_lock_);  //!< Synchronisation

	if (phi_ok_)
	{
        y = phi_.at(t);
		return SUCCESS;
	}
	else
	{
		INDICATE_FAILURE
		;
		return MMB_NIN;
	}
}

kinematica::KinematicScene_ptr exotica::TaskMap::getScene()
{
    return scene_;
}

exotica::EReturn exotica::TaskMap::jacobian(Eigen::Ref<Eigen::MatrixXd> J, int t)
{
	LOCK(jac_lock_);

	if (jac_ok_)
	{
        if(J.rows()!=jac_.at(t).rows() || J.cols()!=jac_.at(t).cols())
        {
            ERROR("Jacoban has wrong size!");
            return FAILURE;
        }
        else
        {
            for(int i=0;i<jac_[t].rows();i++)
                for(int j=0;j<jac_[t].cols();j++)
                    J(i,j) = jac_[t](i,j);
        }
		return SUCCESS;
	}
	else
	{
		INDICATE_FAILURE
		;
		return MMB_NIN;
	}
}

exotica::EReturn exotica::TaskMap::setPhi(const Eigen::Ref<const Eigen::VectorXd> & y, int t)
{
	LOCK(phi_lock_);

    phi_.at(t) = y;
	phi_ok_ = true;
	return SUCCESS;
}

exotica::EReturn exotica::TaskMap::setJacobian(const Eigen::Ref<const Eigen::MatrixXd> & J, int t)
{
	LOCK(jac_lock_);

    jac_.at(t) = J;
	jac_ok_ = true;
	return SUCCESS;
}

void exotica::TaskMap::invalidate()
{
	LOCK(phi_lock_);
	LOCK(jac_lock_);
	phi_ok_ = jac_ok_ = false;
}

exotica::EReturn exotica::TaskMap::setTimeSteps(const int T)
{
    phi_.resize(T);
    jac_.resize(T);
    return SUCCESS;
}
