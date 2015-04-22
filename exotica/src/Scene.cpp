/*
 * Scene.cpp
 *
 *  Created on: 10 Mar 2015
 *      Author: yiming
 */

#include "exotica/Scene.h"

namespace fcl_convert
{
	void fcl2Eigen(const fcl::Vec3f & fcl, Eigen::Vector3d & eigen)
	{
		eigen(0) = fcl.data.vs[0];
		eigen(1) = fcl.data.vs[1];
		eigen(2) = fcl.data.vs[2];
	}

	void fcl2Eigen(const fcl::Transform3f & fcl, Eigen::Vector3d & eigen)
	{
		eigen(0) = fcl.getTranslation().data.vs[0];
		eigen(1) = fcl.getTranslation().data.vs[1];
		eigen(2) = fcl.getTranslation().data.vs[2];
	}
}
namespace exotica
{
	///////////////////////////////////////////////////////////////
	///////////////////////	Collision Scene	///////////////////////
	///////////////////////////////////////////////////////////////
	CollisionScene::CollisionScene() :
			compute_dist(true)
	{
		fcl_robot_.clear();
		fcl_world_.clear();
		geo_robot_.clear();
		geo_world_.clear();
	}

	CollisionScene::~CollisionScene()
	{
		//TODO
	}

	EReturn CollisionScene::initialise(const planning_scene::PlanningSceneConstPtr & ps,
			const std::vector<std::string> & joints, std::string & mode)
    {        
		ps_.reset(new planning_scene::PlanningScene(ps->getRobotModel()));
		moveit_msgs::PlanningScene msg;
		ps->getPlanningSceneMsg(msg);
		ps_->setPlanningSceneMsg(msg);
        ps_->getCurrentStateNonConst().update(true);
        const std::vector<const robot_model::LinkModel*>& links = ps_->getCollisionRobot()->getRobotModel()->getLinkModelsWithCollisionGeometry();

        for (std::size_t i = 0; i < links.size(); ++i)
        {
            geo_robot_[links[i]->getName()] = geos_ptr(0);
            fcl_robot_[links[i]->getName()] = fcls_ptr(0);
            for (std::size_t j = 0; j < links[i]->getShapes().size(); ++j)
            {
                collision_detection::FCLGeometryConstPtr g =
                        collision_detection::createCollisionGeometry(links[i]->getShapes()[j], links[i], j);
                if (g)
                {
                    geo_robot_.at(links[i]->getName()).push_back(g);
                    fcl::CollisionObject *tmp =
                            new fcl::CollisionObject(g->collision_geometry_, collision_detection::transform2fcl(ps_->getCurrentState().getCollisionBodyTransform(g->collision_geometry_data_->ptr.link, g->collision_geometry_data_->shape_index)));
                    fcl_robot_.at(links[i]->getName()).push_back(boost::shared_ptr<
                            fcl::CollisionObject>(tmp));

                }
                else
                    ERROR("Unable to construct collision geometry for link "<< links[i]->getName().c_str());
            }
        }

		joint_index_.resize(joints.size());

        for (std::size_t i = 0; i < ps_->getCurrentState().getVariableNames().size(); i++)
		{
			for (std::size_t j = 0; j < joints.size(); j++)
			{
                if (ps_->getCurrentState().getVariableNames()[i] == joints[j])
				{
					joint_index_[j] = i;
					break;
				}
			}
		}

		if (mode.compare("Sampling") == 0)
			compute_dist = false;
		return SUCCESS;
	}

	EReturn CollisionScene::update(const Eigen::VectorXd x)
	{
		for (std::size_t i = 0; i < joint_index_.size(); i++)
			ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i], x(i));
		ps_->getCurrentStateNonConst().update(true);

		if (compute_dist)
		{
			for (auto & it : fcl_robot_)
				for (std::size_t i = 0; i < it.second.size(); ++i)
				{
					collision_detection::CollisionGeometryData* cd =
							static_cast<collision_detection::CollisionGeometryData*>(it.second[i]->collisionGeometry()->getUserData());
					it.second[i]->setTransform(collision_detection::transform2fcl(ps_->getCurrentState().getCollisionBodyTransform(cd->ptr.link, cd->shape_index)));
					it.second[i]->getTransform().transform(it.second[i]->collisionGeometry()->aabb_center);
				}

			collision_detection::WorldConstPtr tmp_world = ps_->getCollisionWorld()->getWorld();
			std::vector<std::string> obj_id_ = tmp_world->getObjectIds();
			fcl_world_.clear();
			if (obj_id_.size() > 0)
			{
				for (std::size_t i = 0; i < obj_id_.size(); ++i)
				{
					std::size_t index_size = tmp_world->getObject(obj_id_[i])->shapes_.size();
					fcl_world_[obj_id_[i]] = fcls_ptr(index_size);
					for (std::size_t j = 0; j < index_size; j++)
					{
						collision_detection::FCLGeometryConstPtr g =
								collision_detection::createCollisionGeometry(tmp_world->getObject(obj_id_[i])->shapes_[j], tmp_world->getObject(obj_id_[i]).get());
						boost::shared_ptr<fcl::CollisionObject> obj =
								boost::shared_ptr<fcl::CollisionObject>(new fcl::CollisionObject(g->collision_geometry_, collision_detection::transform2fcl(tmp_world->getObject(obj_id_[i])->shape_poses_[j])));
						obj->getTransform().transform(obj->collisionGeometry()->aabb_center);
						fcl_world_.at(obj_id_[i])[j] = obj;
					}
				}
			}
		}
		return SUCCESS;
	}

	EReturn CollisionScene::getDistance(const std::string & o1, const std::string & o2, double d)
	{
		fcls_ptr fcl1, fcl2;
		if (fcl_robot_.find(o1) != fcl_robot_.end())
			fcl1 = fcl_robot_.at(o1);
		else if (fcl_world_.find(o1) != fcl_world_.end())
			fcl1 = fcl_world_.at(o1);
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (fcl_world_.find(o2) != fcl_world_.end())
			fcl2 = fcl_world_.at(o2);
		else if (fcl_robot_.find(o2) != fcl_robot_.end())
			fcl2 = fcl_robot_.at(o2);
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		fcl::DistanceRequest req(false);
		fcl::DistanceResult res;
		d = distance(fcl1, fcl2, req, res);
		return SUCCESS;
	}
	EReturn CollisionScene::getDistance(const std::string & o1, const std::string & o2, double d,
			Eigen::Vector3d & p1, Eigen::Vector3d & p2)
	{
		fcls_ptr fcl1, fcl2;
		if (fcl_robot_.find(o1) != fcl_robot_.end())
			fcl1 = fcl_robot_.at(o1);
		else if (fcl_world_.find(o1) != fcl_world_.end())
			fcl1 = fcl_world_.at(o1);
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (fcl_world_.find(o2) != fcl_world_.end())
			fcl2 = fcl_world_.at(o2);
		else if (fcl_robot_.find(o2) != fcl_robot_.end())
			fcl2 = fcl_robot_.at(o2);
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		fcl::DistanceRequest req(true);
		fcl::DistanceResult res;
		if (distance(fcl1, fcl2, req, res) >= 0)
		{
			d = res.min_distance;
			fcl_convert::fcl2Eigen(res.nearest_points[0], p1);
			fcl_convert::fcl2Eigen(res.nearest_points[1], p2);
		}
		return SUCCESS;
	}

	bool CollisionScene::isStateValid(bool self)
	{
		//	TODO
        return ps_->isStateValid(ps_->getCurrentStateNonConst());
	}

	EReturn CollisionScene::getRobotDistance(const std::string & link, bool self, double & d,
			Eigen::Vector3d & p1, Eigen::Vector3d & p2, Eigen::Vector3d & norm,
			Eigen::Vector3d & c1, Eigen::Vector3d & c2)
	{
		fcls_ptr fcl_link;
		if (fcl_robot_.find(link) != fcl_robot_.end())
			fcl_link = fcl_robot_.at(link);
		else
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		d = 9999;
		fcl::DistanceRequest req(true);
		fcl::DistanceResult res;
		fcl_convert::fcl2Eigen(fcl_link[0]->getTransform().transform(fcl_link[0]->collisionGeometry()->aabb_center), c1);
		if (self)
		{
			for (auto & it : fcl_robot_)
			{
				for (std::size_t i = 0; i < it.second.size(); i++)
				{
					if (distance(fcl_link, it.second, req, res) < 0)
					{
						INDICATE_FAILURE
						d = -1;
						return WARNING;
					}
					else if (res.min_distance < d)
					{
						d = res.min_distance;
						fcl_convert::fcl2Eigen(it.second[i]->getTransform().transform(it.second[i]->collisionGeometry()->aabb_center), c2);
					}
				}
			}
		}
		for (auto & it : fcl_world_)
		{
			for (std::size_t i = 0; i < it.second.size(); i++)
			{
				if (distance(fcl_link, it.second, req, res) < 0)
				{
					d = -1;
					return WARNING;
				}
				else if (res.min_distance < d)
				{
					d = res.min_distance;

					fcl_convert::fcl2Eigen(it.second[i]->getTransform().transform(it.second[i]->collisionGeometry()->aabb_center), c2);
				}
			}
		}

		fcl_convert::fcl2Eigen(res.nearest_points[0], p1);
		fcl_convert::fcl2Eigen(res.nearest_points[1], p2);

		//	Bugs for non-mesh obstacles. TODO
//		KDL::Frame tmp1 = KDL::Frame(KDL::Vector(p1(0), p1(1), p1(2)));
//		//tmp1 = KDL::Frame(KDL::Vector(c1(0), c1(1), c1(2)))*tmp1.Inverse();
//		KDL::Frame tmp2 = KDL::Frame(KDL::Vector(p2(0), p2(1), p2(2)));
//		tmp2 = tmp2 * KDL::Frame(KDL::Vector(c2(0), c2(1), c2(2)));
//
//		p1(0)=tmp1.p.data[0];
//		p1(1)=tmp1.p.data[1];
//		p1(2)=tmp1.p.data[2];
//		p2(0)=tmp2.p.data[0];
//		p2(1)=tmp2.p.data[1];
//		p2(2)=tmp2.p.data[2];
		norm = p2 - p1;
		return SUCCESS;
	}
	double CollisionScene::distance(const fcls_ptr & fcl1, const fcls_ptr & fcl2,
			const fcl::DistanceRequest & req, fcl::DistanceResult & res)
	{
		for (int i = 0; i < fcl1.size(); i++)
		{
			for (int j = 0; j < fcl2.size(); j++)
			{
				if (fcl1[i] == nullptr)
				{
					INDICATE_FAILURE
				}
				if (fcl2[j] == nullptr)
				{
					INDICATE_FAILURE
				}
				if (fcl::distance(fcl1[i].get(), fcl2[j].get(), req, res) < 0)
				{
					res.min_distance = -1;
					return -1;
				}
			}
		}
		return res.min_distance;
	}

	const robot_state::RobotState& CollisionScene::getCurrentState()
	{
        ps_->getCurrentStateNonConst().update(true);
		return ps_->getCurrentState();
	}

	const planning_scene::PlanningScenePtr CollisionScene::getPlanningScene()
	{
		return ps_;
	}
///////////////////////////////////////////////////////////////
///////////////////////	EXOTica Scene	///////////////////////
///////////////////////////////////////////////////////////////

	Scene::Scene(const std::string & name) :
            name_(name), nh_(name + "_node"), N(0), initialised_(false), update_jacobians_(true)
	{
		eff_names_.clear();
		eff_offsets_.clear();
		phis_.clear();
		jacs_.clear();
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
		LOCK(lock_);
		if (!handle.FirstChildElement("Kinematica").ToElement())
		{
			INDICATE_FAILURE
			return FAILURE;
		}

        if (!ok(server->getModel("robot_description",model_)))
        {
            ROS_ERROR_STREAM("Could not load robot model from 'robot_description' parameter!");
            return FAILURE;
        }

		tinyxml2::XMLHandle kinematica_handle(handle.FirstChildElement("Kinematica"));
        if (!kinematica_.initKinematics(kinematica_handle,model_->getURDF().get()))
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		N = kinematica_.getNumJoints();
		collision_scene_.reset(new CollisionScene());

		EParam<std_msgs::String> tmp;
		server->getParam("/PlanningMode", tmp);
		mode_ = tmp->data;
        if (mode_.compare("Sampling")==0)
            update_jacobians_ = false;
#ifdef EXOTICA_DEBUG_MODE
		state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("disp_state", 100);
		ROS_ERROR_STREAM("Running in debug mode, a robot state will be published to '"<<name_<<"_node/disp_state'");
#endif
        {
            planning_scene::PlanningScenePtr tmp(new planning_scene::PlanningScene(model_));

            if(!ok(collision_scene_->initialise(tmp, kinematica_.getJointNames(), mode_)))
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }

		return SUCCESS;
	}

	EReturn Scene::getForwardMap(const std::string & task, Eigen::Ref<Eigen::VectorXd> phi)
	{
		LOCK(lock_);
		if (phis_.find(task) == phis_.end())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
        Eigen::Ref<Eigen::VectorXd> y(*(phis_.at(task)));
        for(int r=0;r<phi.rows();r++)
        {
            phi(r) = y(r);
        }
		return SUCCESS;
	}

	EReturn Scene::getJacobian(const std::string & task, Eigen::Ref<Eigen::MatrixXd> jac)
	{
		LOCK(lock_);
		if (jacs_.find(task) == jacs_.end())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
        Eigen::Ref<Eigen::MatrixXd> J(*(jacs_.at(task)));
        for(int r=0;r<jac.rows();r++)
        {
            for(int c=0;c<jac.cols();c++)
            {
                jac(r,c) = J(r,c);
            }
        }
		return SUCCESS;
	}

	EReturn Scene::appendTaskMap(const std::string & name, const std::vector<std::string> & eff,
			const std::vector<KDL::Frame> & offset)
	{
		LOCK(lock_);
		eff_names_[name] = eff;
		eff_offsets_[name] = offset;

		return SUCCESS;
	}

    EReturn Scene::getPoses(const std::vector<std::string> names, std::vector<KDL::Frame> & poses)
    {
        LOCK(lock_);
        poses.resize(names.size());
        for(int i=0;i<names.size();i++)
        {
            if(!kinematica_.getPose(names[i],poses[i]))
            {
                poses.resize(0);
                INDICATE_FAILURE;
                return FAILURE;
            }

        }
        return SUCCESS;
    }

    EReturn Scene::updateEndEffectors(const std::string & task,
			const std::vector<KDL::Frame> & offset)
	{
        LOCK(lock_);
        if (eff_index_.find(task) == eff_index_.end())
        {
            INDICATE_FAILURE
            ROS_ERROR_STREAM("Task name: '"<<task<<"'\n"<<eff_index_.size());
            return FAILURE;
        }
        if (offset.size() != eff_index_.at(task).size())
        {
            INDICATE_FAILURE
            return FAILURE;
        }
        if (!kinematica_.updateEndEffectorOffsets(eff_index_.at(task), offset))
        {
            INDICATE_FAILURE
            return FAILURE;
        }
        return SUCCESS;
	}

	EReturn Scene::activateTaskMaps()
	{

		LOCK(lock_);
		kinematica::SolutionForm_t tmp_sol;
        tmp_sol.end_effector_segs.clear();
		for (auto & it : eff_names_)
		{
			for (int i = 0; i < it.second.size(); i++)
			{
				tmp_sol.end_effector_segs.push_back(it.second[i]);
			}
		}

        tmp_sol.end_effector_offs.clear();
		for (auto & it : eff_offsets_)
		{
			for (int i = 0; i < it.second.size(); i++)
			{
				tmp_sol.end_effector_offs.push_back(it.second[i]);
			}
		}

		if (!kinematica_.updateEndEffectors(tmp_sol))
        {
            INDICATE_FAILURE;
			return FAILURE;
        }
		std::vector<int> tmp_index;
		if (!kinematica_.getEndEffectorIndex(tmp_index))
        {
            INDICATE_FAILURE;
			return FAILURE;
        }
        Phi_.setZero(3 * kinematica_.getEffSize());
        Jac_.setZero(3 * kinematica_.getEffSize(), N);
		int tmp_size = 0, tmp_eff_size = 0;
        phis_.clear();
        jacs_.clear();
        eff_index_.clear();
		for (auto & it : eff_names_)
		{
			eff_index_[it.first] =
					std::vector<int>(tmp_index.begin() + tmp_eff_size, tmp_index.begin()
							+ tmp_eff_size + it.second.size());
            phis_[it.first] = Eigen::VectorXdRef_ptr(Phi_.segment(tmp_size, 3 * it.second.size()));
            jacs_[it.first] = Eigen::MatrixXdRef_ptr(Jac_.block(tmp_size, 0, 3 * it.second.size(), N));
			tmp_size += 3 * it.second.size();
			tmp_eff_size += it.second.size();
		}

		initialised_ = true;

		return SUCCESS;
	}

	EReturn Scene::update(const Eigen::VectorXd x, const int t)
	{
		LOCK(lock_);
		if (!initialised_)
		{
			ERROR("EXOTica scene needs to be initialised via 'activateTaskMaps()'.");
			INDICATE_FAILURE
			return FAILURE;
		}
        else
        {
            if (ok(collision_scene_->update(x)))
            {
                if (kinematica_.updateConfiguration(x))
                {
                    if (kinematica_.generateForwardMap(Phi_))
                    {
                        if (update_jacobians_)
                        {
                            if (kinematica_.generateJacobian(Jac_))
                            {
                                // All is fine
                            }
                            else
                            {
                                INDICATE_FAILURE;
                                return FAILURE;
                            }
                        }
                        // else Also fine, just skip computing the Jacobians
                    }
                    else
                    {
                        INDICATE_FAILURE;
                        return FAILURE;
                    }
                }
                else
                {
                    INDICATE_FAILURE;
                    return FAILURE;
                }
            }
            else
            {
                INDICATE_FAILURE;
                return FAILURE;
            }
        }

#ifdef EXOTICA_DEBUG_MODE
		moveit_msgs::DisplayRobotState msg;
		robot_state::robotStateToRobotStateMsg(collision_scene_->getCurrentState(), msg.state);
		state_pub_.publish(msg);
		ros::spinOnce();
#endif

		return SUCCESS;
	}

	EReturn Scene::setCollisionScene(const planning_scene::PlanningSceneConstPtr & scene)
	{

		return collision_scene_->initialise(scene, kinematica_.getJointNames(), mode_);
	}

	EReturn Scene::setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene)
	{

		planning_scene::PlanningScenePtr tmp;
		tmp.reset(new planning_scene::PlanningScene(model_));
		tmp->setPlanningSceneMsg(*scene.get());

		EParam<std_msgs::String> mode;

		return collision_scene_->initialise(tmp, kinematica_.getJointNames(), mode_);
	}

	int Scene::getNumJoints()
	{
		LOCK(lock_);
		return N;
	}

	CollisionScene_ptr & Scene::getCollisionScene()
	{
		return collision_scene_;
	}

	EReturn Scene::getEndEffectors(const std::string & task, std::vector<std::string> & effs)
	{
		if (eff_names_.find(task) == eff_names_.end())
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		effs = eff_names_.at(task);
		return SUCCESS;
	}

	int Scene::getMapSize(const std::string & task)
	{
		LOCK(lock_);
		if (eff_names_.find(task) == eff_names_.end())
			return -1;
		return eff_names_.at(task).size();
    }

    EReturn Scene::getCoMProperties(std::string& task, std::vector<std::string> & segs, Eigen::VectorXd & mass,
			std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
			std::vector<KDL::Frame> & base_pose)
	{
		LOCK(lock_);
        if (eff_index_.find(task) == eff_index_.end())
        {
            INDICATE_FAILURE
            return FAILURE;
        }
        if (kinematica_.getCoMProperties(eff_index_.at(task), segs, mass, cog, tip_pose, base_pose))
        {
            return SUCCESS;
        }
        else
        {
            INDICATE_FAILURE;
            return FAILURE;
        }
	}

	std::string Scene::getRootName()
	{
		LOCK(lock_);
		return kinematica_.getRootName();
	}

	planning_scene::PlanningScenePtr Scene::getPlanningScene()
	{
		return collision_scene_->getPlanningScene();
	}

	kinematica::KinematicTree & Scene::getSolver()
	{
		return kinematica_;
	}
}
//	namespace exotica

