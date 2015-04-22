#include "exotica/TaskMap.h"

exotica::TaskMap::TaskMap()
{
	//!< Just reset the flags
	phi_ok_ = false;
	jac_ok_ = false;
}

exotica::Scene_ptr exotica::TaskMap::getScene()
{
    return scene_;
}

exotica::EReturn exotica::TaskMap::initBase(tinyxml2::XMLHandle & handle, Server_ptr & server,
		const Scene_map & scene_ptr)
{
	//!< Clear flags and kinematic scene pointer
	Object::initBase(handle, server);
	invalidate();
	if (!server)
	{
		INDICATE_FAILURE
		return FAILURE;
	}
	server_ = server;
	scene_ = Scene_ptr();  //!< Null pointer

	if (handle.FirstChildElement("Scene").ToElement())
	{
		LOCK(scene_lock_);  //!< Local lock
		const char * name = handle.FirstChildElement("Scene").ToElement()->Attribute("name");
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
        return PAR_ERR;
	}

	std::vector<std::string> tmp_eff(0);
	std::vector<KDL::Frame> tmp_offset(0);

	tinyxml2::XMLHandle segment_handle(handle.FirstChildElement("EndEffector").FirstChildElement("limb"));
	while (segment_handle.ToElement())
	{
		if (!segment_handle.ToElement()->Attribute("segment"))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		tmp_eff.push_back(segment_handle.ToElement()->Attribute("segment"));
		KDL::Frame temp_frame = KDL::Frame::Identity(); //!< Initialise to identity
		if (segment_handle.FirstChildElement("vector").ToElement())
		{
			Eigen::VectorXd temp_vector;
			if (!ok(getVector(*(segment_handle.FirstChildElement("vector").ToElement()), temp_vector)))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			if (temp_vector.size() != 3)
			{
				return FAILURE;
			}
			temp_frame.p.x(temp_vector(0));
			temp_frame.p.y(temp_vector(1));
			temp_frame.p.z(temp_vector(2));
		}
		if (segment_handle.FirstChildElement("quaternion").ToElement())
		{
			Eigen::VectorXd temp_vector;
			if (!ok(getVector(*(segment_handle.FirstChildElement("quaternion").ToElement()), temp_vector)))
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			if (temp_vector.size() != 4)
			{
				INDICATE_FAILURE
				return FAILURE;
			}
			temp_frame.M =
					KDL::Rotation::Quaternion(temp_vector(1), temp_vector(2), temp_vector(3), temp_vector(0));
		}
		tmp_offset.push_back(temp_frame);
		segment_handle = segment_handle.NextSiblingElement("limb");
	}

    scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
    if(ok(initDerived(handle)))
    {
        return exotica::SUCCESS;
    }
    else
    {
        ERROR("Failed to initialise task '"<<getObjectName() <<"'");
        return exotica::FAILURE;
    }
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

exotica::EReturn exotica::TaskMap::jacobian(Eigen::Ref<Eigen::MatrixXd> J, int t)
{
	LOCK(jac_lock_);

	if (jac_ok_)
	{
		if (J.rows() != jac_.at(t).rows() || J.cols() != jac_.at(t).cols())
		{
			ERROR("Jacoban has wrong size!");
			return FAILURE;
		}
		else
		{
			for (int i = 0; i < jac_[t].rows(); i++)
				for (int j = 0; j < jac_[t].cols(); j++)
					J(i, j) = jac_[t](i, j);
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
