/*
 * Scene.cpp
 *
 *  Created on: 10 Mar 2015
 *      Author: yiming
 */

#include "exotica/Scene.h"
namespace exotica
{
	Scene::Scene(const std::string & name) :
			name_(name), N(0)
	{
		eff_names_.clear();
		eff_offsets_.clear();
		phis_.clear();
		jacs_.clear();
		//TODO
	}

	Scene::~Scene()
	{
		//TODO
	}

	std::string Scene::getName()
	{
		return name_;
	}

	EReturn Scene::initialisation(tinyxml2::XMLHandle & handle, const Server_ptr & server)
	{

		if (!handle.FirstChildElement("Kinematica").ToElement())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		tinyxml2::XMLHandle kinematica_handle(handle.FirstChildElement("Kinematica"));
		if (!kinematica_.initKinematics(kinematica_handle))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		N = kinematica_.getNumJoints();

		return SUCCESS;
	}

	EReturn Scene::getForwardMap(const std::string & task, Eigen::Ref<Eigen::VectorXd> phi)
	{
		if (phis_.find(task) == phis_.end())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		phi = *phis_.at(task);
		return SUCCESS;
	}

	EReturn Scene::getJacobian(const std::string & task, Eigen::Ref<Eigen::MatrixXd> jac)
	{
		if (jacs_.find(task) == jacs_.end())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		jac = *jacs_.at(task);
		return SUCCESS;
	}

	EReturn Scene::appendTaskMap(const std::string & name, const std::vector<std::string> & eff,
			const std::vector<KDL::Frame> & offset)
	{
		eff_names_[name] = eff;
		eff_offsets_[name] = offset;
		return SUCCESS;
	}

	EReturn Scene::updateEndEffectors(const std::string & task,
			const std::vector<KDL::Frame> & offset)
	{
		std::cerr << "Scene UpdateEndEffectors has not been implemented yet!!!!" << std::endl;
		INDICATE_FAILURE
		return FAILURE;
	}

	EReturn Scene::activateTaskMaps()
	{
		kinematica::SolutionForm_t tmp_sol;
		for (auto & it : eff_names_)
		{
			eff_index_[it.first] = std::vector<int>(0);
			for (int i = 0; i < it.second.size(); i++)
			{
				eff_index_[it.first].push_back(tmp_sol.end_effector_segs.size());
				tmp_sol.end_effector_segs.push_back(it.second[i]);
			}
		}

		for (auto & it : eff_offsets_)
		{
			for (int i = 0; i < it.second.size(); i++)
			{
				tmp_sol.end_effector_offs.push_back(it.second[i]);
			}
		}

		if (!kinematica_.updateEndEffectors(tmp_sol))
			return FAILURE;
		Phi_.setZero(3 * eff_index_.size());
		Jac_.setZero(3 * kinematica_.getEffSize(), N);
		int tmp_size = 0;
		for (auto & it : eff_index_)
		{
			std::cout << "tmp " << tmp_size << " size=" << 3 * kinematica_.getEffSize() << std::endl;
			phis_[it.first] = boost::shared_ptr<Eigen::Ref<Eigen::VectorXd> >(new Eigen::Ref<
					Eigen::VectorXd>(Phi_.segment(tmp_size, 3 * eff_index_.at(it.first).size())));
			jacs_[it.first] = boost::shared_ptr<Eigen::Ref<Eigen::MatrixXd> >(new Eigen::Ref<
					Eigen::MatrixXd>(Jac_.block(tmp_size, 0, 3 * it.second.size(), N)));
			tmp_size += 3 * it.second.size();
		}

		return SUCCESS;
	}

	EReturn Scene::update(const Eigen::VectorXd x, const int t)
	{
		LOCK(lock_);
		if (!kinematica_.updateConfiguration(x))
			return FAILURE;
		if (!kinematica_.generateForwardMap())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (!kinematica_.generateJacobian())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (!kinematica_.getPhi(Phi_) || !kinematica_.getJacobian(Jac_))
			return FAILURE;
		return SUCCESS;
	}

	int Scene::getNumJoints()
	{
		return N;
	}

	int Scene::getMapSize(const std::string & task)
	{
		if (eff_names_.find(task) == eff_names_.end())
			return -1;
		return eff_names_.at(task).size();
	}

	EReturn Scene::getCoMProperties(std::vector<std::string> & segs, Eigen::VectorXd & mass,
			std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
			std::vector<KDL::Frame> & base_pose)
	{
		if (!kinematica_.getCoMProperties(segs, mass, cog, tip_pose, base_pose))
			return FAILURE;
		return SUCCESS;
	}

	std::string Scene::getRootName()
	{
		return kinematica_.getRootName();
	}
	planning_scene::PlanningScenePtr Scene::getPlanningScene()
	{
		INDICATE_FAILURE
		return nullptr;
	}
	kinematica::KinematicTree & Scene::getSolver()
	{
		INDICATE_FAILURE
		return kinematica_;
	}
}
//	namespace exotica

