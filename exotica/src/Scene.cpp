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

  void fcl2EigenTranslation(const fcl::Vec3f & fcl, Eigen::Vector3d & eigen)
  {
    eigen(0) = fcl.data.vs[0];
    eigen(1) = fcl.data.vs[1];
    eigen(2) = fcl.data.vs[2];
  }
}
namespace exotica
{
///////////////////////////////////////////////////////////////
///////////////////////	Collision Scene	///////////////////////
///////////////////////////////////////////////////////////////
  CollisionScene::CollisionScene(const Server_ptr & server,
      const std::string & scene_name)
      : server_(server), compute_dist(true), stateCheckCnt_(0), scene_name_(
          scene_name)
  {
  }

  CollisionScene::~CollisionScene()
  {
    //TODO
  }

  EReturn CollisionScene::reinitialise()
  {
    fcl_robot_.clear();
    fcl_world_.clear();
    geo_robot_.clear();
    geo_world_.clear();
    ps_->getCurrentStateNonConst().update(true);
    const std::vector<const robot_model::LinkModel*>& links =
        ps_->getCollisionRobot()->getRobotModel()->getLinkModelsWithCollisionGeometry();
    acm_ = ps_->getAllowedCollisionMatrix();
    for (std::size_t i = 0; i < links.size(); ++i)
    {
      geo_robot_[links[i]->getName()] = geos_ptr(0);
      fcl_robot_[links[i]->getName()] = fcls_ptr(0);
      for (std::size_t j = 0; j < links[i]->getShapes().size(); ++j)
      {
        shapes::ShapeConstPtr tmp_shape;
        if (links[i]->getShapes()[j]->type != shapes::MESH)
          tmp_shape = boost::shared_ptr<const shapes::Shape>(
              shapes::createMeshFromShape(links[i]->getShapes()[j].get()));
        else
          tmp_shape = links[i]->getShapes()[j];
        if (!tmp_shape || !tmp_shape.get())
        {
          INDICATE_FAILURE
          return FAILURE;
        }
        collision_detection::FCLGeometryConstPtr g =
            collision_detection::createCollisionGeometry(tmp_shape, links[i],
                j);
        if (g)
        {
          geo_robot_.at(links[i]->getName()).push_back(g);
          fcl::CollisionObject *tmp = new fcl::CollisionObject(
              g->collision_geometry_,
              collision_detection::transform2fcl(
                  ps_->getCurrentState().getCollisionBodyTransform(
                      g->collision_geometry_data_->ptr.link,
                      g->collision_geometry_data_->shape_index)));
          fcl_robot_.at(links[i]->getName()).push_back(
              boost::shared_ptr<fcl::CollisionObject>(tmp));

        }
        else
          ERROR(
              "Unable to construct collision geometry for link "<< links[i]->getName().c_str());
      }
    }

    collision_detection::WorldConstPtr tmp_world =
        ps_->getCollisionWorld()->getWorld();
    std::vector<std::string> obj_id_ = tmp_world->getObjectIds();
    if (obj_id_.size() > 0)
    {
      for (std::size_t i = 0; i < obj_id_.size(); ++i)
      {
        std::size_t index_size =
            tmp_world->getObject(obj_id_[i])->shapes_.size();
        fcl_world_[obj_id_[i]] = fcls_ptr(0);
        geo_world_[obj_id_[i]] = geos_ptr(0);
        trans_world_[obj_id_[i]] = std::vector<fcl::Transform3f>(0);
        for (std::size_t j = 0; j < index_size; j++)
        {
          shapes::ShapeConstPtr tmp_shape;

          if (tmp_world->getObject(obj_id_[i])->shapes_[j]->type
              == shapes::OCTREE)
          {
            tmp_world->getObject(obj_id_[i])->shapes_[j]->print();
            tmp_shape = boost::static_pointer_cast<const shapes::Shape>(
                tmp_world->getObject(obj_id_[i])->shapes_[j]);
          }
          else
          {
            if (tmp_world->getObject(obj_id_[i])->shapes_[j]->type
                != shapes::MESH)
            {
              tmp_shape = boost::shared_ptr<const shapes::Shape>(
                  shapes::createMeshFromShape(
                      tmp_world->getObject(obj_id_[i])->shapes_[j].get()));
            }
            else
            {
              tmp_shape = tmp_world->getObject(obj_id_[i])->shapes_[j];
            }
          }

          if (!tmp_shape || !tmp_shape.get())
          {
            INDICATE_FAILURE
            return FAILURE;
          }
          collision_detection::FCLGeometryConstPtr g =
              collision_detection::createCollisionGeometry(tmp_shape,
                  tmp_world->getObject(obj_id_[i]).get());
          geo_world_.at(obj_id_[i]).push_back(g);
          trans_world_.at(obj_id_[i]).push_back(
              fcl::Transform3f(
                  collision_detection::transform2fcl(
                      tmp_world->getObject(obj_id_[i])->shape_poses_[j])));
          fcl_world_.at(obj_id_[i]).push_back(
              boost::shared_ptr<fcl::CollisionObject>(
                  new fcl::CollisionObject(g->collision_geometry_,
                      collision_detection::transform2fcl(
                          tmp_world->getObject(obj_id_[i])->shape_poses_[j]))));
        }
      }
    }
    return SUCCESS;
  }

  EReturn CollisionScene::initialise(
      const moveit_msgs::PlanningSceneConstPtr & msg,
      const std::vector<std::string> & joints, std::string & mode,
      BASE_TYPE base_type)
  {
    ps_.reset(
        new planning_scene::PlanningScene(
            server_->getModel("robot_description")));
    ps_->setPlanningSceneMsg(*msg.get());
    base_type_ = base_type;
    if (ok(reinitialise()))
    {
      joint_index_.resize(joints.size());

      for (std::size_t i = 0;
          i < ps_->getCurrentState().getVariableNames().size(); i++)
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
      {
        compute_dist = false;
        INFO("Computing distance in Collision scene is Disabled");
      }
      else
        INFO("Computing distance in Collision scene is Enabled");
      return SUCCESS;
    }
    else
    {
      return FAILURE;
    }
  }

  EReturn CollisionScene::updateWorld(
      const moveit_msgs::PlanningSceneWorldConstPtr & world)
  {
//		for (int i=0;i<world->collision_objects.size();i++)
//		{
//			if(trans_world_.find(world->collision_objects[i].id) != trans_world_.end())
//			{
//				std::map<std::string, fcls_ptr>::iterator it = trans_world_.find(world->collision_objects[i].id);
//				for(int j =0;j<it->second.size();j++)
//				{
//					it->second[j] = fcl::Transform3f(collision_detection::transform2fcl(world->collision_objects[i].mesh_poses[j]));
//				}
//			}
//		}
    return SUCCESS;
  }

  EReturn CollisionScene::update(Eigen::VectorXdRefConst x)
  {
    if (joint_index_.size() != x.rows())
    {
      ERROR(
          "Size does not match, need vector size of "<<joint_index_.size()<<" but "<<x.rows()<<" is provided");
      return FAILURE;
    }

    if (base_type_ == BASE_TYPE::FIXED)
    {
      for (std::size_t i = 0; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    else if (base_type_ == BASE_TYPE::FLOATING)
    {
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/trans_x",
          x(0));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/trans_y",
          x(1));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/trans_z",
          x(2));
      KDL::Rotation rot = KDL::Rotation::EulerZYX(x(3), x(4), x(5));
      Eigen::VectorXd quat(4);
      rot.GetQuaternion(quat(0), quat(1), quat(2), quat(3));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/rot_x",
          quat(0));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/rot_y",
          quat(1));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/rot_z",
          quat(2));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/rot_w",
          quat(3));
      for (std::size_t i = 6; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    else if (base_type_ == BASE_TYPE::PLANAR)
    {
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/x", x(0));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/y", x(1));
      ps_->getCurrentStateNonConst().setVariablePosition("world_joint/theta",
          x(2));
    }
    ps_->getCurrentStateNonConst().update(true);
    if (compute_dist)
    {
      for (auto & it : fcl_robot_)
        for (std::size_t i = 0; i < it.second.size(); ++i)
        {
          collision_detection::CollisionGeometryData* cd =
              static_cast<collision_detection::CollisionGeometryData*>(it.second[i]->collisionGeometry()->getUserData());
          it.second[i]->setTransform(
              collision_detection::transform2fcl(
                  ps_->getCurrentState().getCollisionBodyTransform(cd->ptr.link,
                      cd->shape_index)));
          it.second[i]->getTransform().transform(
              it.second[i]->collisionGeometry()->aabb_center);
        }
    }
    return SUCCESS;
  }

  EReturn CollisionScene::getDistance(const std::string & o1,
      const std::string & o2, double& d, double safeDist)
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
    d = distance(fcl1, fcl2, req, res, safeDist);
    return SUCCESS;
  }
  EReturn CollisionScene::getDistance(const std::string & o1,
      const std::string & o2, double& d, Eigen::Vector3d & p1,
      Eigen::Vector3d & p2, double safeDist)
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
    if (distance(fcl1, fcl2, req, res, safeDist) >= 0)
    {
      d = res.min_distance;
      fcl_convert::fcl2Eigen(res.nearest_points[0], p1);
      fcl_convert::fcl2Eigen(res.nearest_points[1], p2);
    }
    return SUCCESS;
  }

  bool CollisionScene::isStateValid(bool self)
  {
    stateCheckCnt_++;
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    ps_->getCollisionWorld()->checkRobotCollision(req,res,*ps_->getCollisionRobot(),ps_->getCurrentState());
//    ERROR(ps_->getCurrentState().getCollisionBodyTransform("leftForearmLink",0).translation().transpose());
//    if(res.collision){
//      HIGHLIGHT("Contact "<<res.contacts.begin()->first.first<<" "<<res.contacts.begin()->first.second<<" at "<<res.contacts.begin()->second[0].pos.transpose());
//    }
    return !res.collision;
  }

  bool CollisionScene::isStateValid(const Eigen::VectorXd &q, bool self)
  {
    update(q);
    return ps_->isStateValid(ps_->getCurrentState());
  }

  EReturn CollisionScene::getRobotDistance(const std::string & link, bool self,
      double & d, Eigen::Vector3d & p1, Eigen::Vector3d & p2,
      Eigen::Vector3d & norm, Eigen::Vector3d & c1, Eigen::Vector3d & c2,
      double safeDist)
  {
    fcls_ptr fcl_link;
    if (fcl_robot_.find(link) != fcl_robot_.end())
      fcl_link = fcl_robot_.at(link);
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    d = INFINITY;
    fcl::DistanceRequest req(true);
    fcl::DistanceResult res;
    res.min_distance = INFINITY;
    {
      fcl::AABB sumAABB;
      for (int i = 0; i < fcl_link.size(); i++)
      {
        fcl_link[i]->computeAABB();
        sumAABB += fcl_link[i]->getAABB();
      }
      fcl_convert::fcl2Eigen(sumAABB.center(), c1);
    }
    if (self)
    {
      for (auto & it : fcl_robot_)
      {
        collision_detection::AllowedCollision::Type type =
            collision_detection::AllowedCollision::ALWAYS;
        if (link.compare(it.first) != 0 && acm_.getEntry(link, it.first, type))
        {
          if (type == collision_detection::AllowedCollision::NEVER)
          {
            ROS_INFO_STREAM_THROTTLE(2,
                "Checking between "<<link<<" and "<<it.first);
            for (std::size_t i = 0; i < it.second.size(); i++)
            {
              if (distance(fcl_link, it.second, req, res, safeDist) < 0)
              {
//							INDICATE_FAILURE
                d = -1;
                return WARNING;
              }
              else if (res.min_distance < d)
              {
                d = res.min_distance;
                fcl_convert::fcl2Eigen(
                    it.second[i]->getTransform().transform(
                        it.second[i]->collisionGeometry()->aabb_center), c2);
              }
            }
          }
          else
          {
            ROS_INFO_STREAM_THROTTLE(2,
                "Ignoring between "<<link<<" and "<<it.first);
          }
        }
      }
    }

    for (auto & it : fcl_world_)
    {
      for (int i = 0; i < it.second.size(); i++)
      {
        it.second[i]->setTransform(trans_world_.at(it.first)[i]);
        it.second[i]->computeAABB();
      }

      for (std::size_t i = 0; i < it.second.size(); i++)
      {
        if (distance(fcl_link, it.second, req, res, safeDist) < 0)
        {
          d = -1;
          fcl_convert::fcl2Eigen(it.second[i]->getAABB().center(), c2);
          return WARNING;
        }
        else if (res.min_distance < d)
        {
          d = res.min_distance;
          fcl_convert::fcl2Eigen(it.second[i]->getAABB().center(), c2);
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
      const fcl::DistanceRequest & req, fcl::DistanceResult & res,
      double safeDist)
  {
    fcl::DistanceResult tmp;
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
        if (fcl2[j]->getAABB().distance(fcl2[j]->getAABB()) < safeDist)
        {
          if (fcl::distance(fcl1[i].get(), fcl2[j].get(), req, tmp) < 0)
          {
            res = tmp;
            res.min_distance = -1;
            return -1;
          }
          else
          {
            if (tmp.min_distance < res.min_distance)
            {
              res = tmp;
            }
          }
        }
      }
    }

    return res.min_distance;
  }

  const robot_state::RobotState& CollisionScene::getCurrentState()
  {
    return ps_->getCurrentState();
  }

  planning_scene::PlanningScenePtr CollisionScene::getPlanningScene()
  {
    return ps_;
  }

  EReturn CollisionScene::getCollisionLinkTranslation(const std::string & name,
      Eigen::Vector3d & translation)
  {
    if (fcl_robot_.find(name) == fcl_robot_.end()) return FAILURE;
    std::map<std::string, fcls_ptr>::iterator it = fcl_robot_.find(name);
    fcl::AABB sumAABB;
    for (int i = 0; i < it->second.size(); i++)
    {
      it->second[i]->computeAABB();
      sumAABB += it->second[i]->getAABB();
    }
    fcl_convert::fcl2Eigen(sumAABB.center(), translation);
    return SUCCESS;
  }

  EReturn CollisionScene::getWorldObjectTranslation(const std::string & name,
      Eigen::Vector3d & translation)
  {
    if (fcl_world_.find(name) == fcl_world_.end()) return FAILURE;
    std::map<std::string, fcls_ptr>::iterator it = fcl_world_.find(name);
    fcl::AABB sumAABB;
    for (int i = 0; i < it->second.size(); i++)
    {
      it->second[i]->computeAABB();
      sumAABB += it->second[i]->getAABB();
    }
    fcl_convert::fcl2Eigen(sumAABB.center(), translation);
    return SUCCESS;
  }

  EReturn CollisionScene::getTranslation(const std::string & name,
      Eigen::Vector3d & translation)
  {
    EReturn ret = getCollisionLinkTranslation(name, translation);
    return ok(ret) ? ret : getWorldObjectTranslation(name, translation);
  }

///////////////////////////////////////////////////////////////
///////////////////////	EXOTica Scene	///////////////////////
///////////////////////////////////////////////////////////////

  Scene::Scene(const std::string & name)
      : name_(name), N(0), initialised_(false), update_jacobians_(true)
  {
    eff_names_.clear();
    eff_offsets_.clear();
    phis_.clear();
    jacs_.clear();
    object_name_ = name_;
  }

  Scene::~Scene()
  {
//TODO
  }

  robot_model::RobotModelPtr Scene::getRobotModel()
  {
    return model_;

  }

  std::string Scene::getName()
  {
    return name_;
  }

  EReturn Scene::initialisation(tinyxml2::XMLHandle & handle,
      const Server_ptr & server)
  {
    LOCK(lock_);
    server_ = server;
    if (!handle.FirstChildElement("Kinematica").ToElement())
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    if (!ok(server->getModel("robot_description", model_)))
    {
      ERROR("Could not load robot model from 'robot_description' parameter!");
      return FAILURE;
    }

    tinyxml2::XMLHandle tmp_handle(handle.FirstChildElement("Kinematica"));
    if (!kinematica_.initKinematics(tmp_handle, model_))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    std::string base_type = kinematica_.getBaseType();
    if (base_type.compare("fixed") == 0)
      base_type_ = BASE_TYPE::FIXED;
    else if (base_type.compare("floating") == 0)
      base_type_ = BASE_TYPE::FLOATING;
    else if (base_type.compare("planar") == 0) base_type_ = BASE_TYPE::PLANAR;

    N = kinematica_.getNumJoints();
    collision_scene_.reset(new CollisionScene(server_, name_));

    tmp_handle = handle.FirstChildElement("PlanningMode");
    if (!ok(server_->registerParam<std_msgs::String>(name_, tmp_handle, mode_)))
    {
      mode_->data = "Optimization";
      WARNING_NAMED(name_,
          "Planning mode not specified, using default: Optimization.");
    }

    update_jacobians_ = mode_->data.compare("Sampling") != 0 ? true : false;

    tmp_handle = handle.FirstChildElement("VisualDebug");
    server_->registerParam<std_msgs::Bool>(name_, tmp_handle, visual_debug_);
    if (visual_debug_->data)
    {
      ps_pub_ = server_->advertise<moveit_msgs::PlanningScene>(
          name_ + "/PlanningScene", 100, true);
      HIGHLIGHT_NAMED(name_,
          "Running in debug mode, planning scene will be published to '"<<server_->getName()<<"/"<<name_<<"/PlanningScene'");
    }
    {
      planning_scene::PlanningScenePtr tmp(
          new planning_scene::PlanningScene(model_));
      moveit_msgs::PlanningScenePtr msg(new moveit_msgs::PlanningScene());
      tmp->getPlanningSceneMsg(*msg.get());
      if (!ok(
          collision_scene_->initialise(msg, kinematica_.getJointNames(),
              mode_->data, base_type_)))
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }
    INFO_NAMED(name_,
        "Exotica Scene initialised, planning mode set to "<<mode_->data);
    return SUCCESS;
  }

  EReturn Scene::getForwardMap(const std::string & task, Eigen::VectorXdRef phi)
  {
    LOCK(lock_);
    if (phis_.find(task) == phis_.end())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    Eigen::Ref<Eigen::VectorXd> y(*(phis_.at(task)));
    for (int r = 0; r < phi.rows(); r++)
    {
      phi(r) = y(r);
    }
    return SUCCESS;
  }

  EReturn Scene::getForwardMap(const std::string & task,
      Eigen::VectorXdRef_ptr& phi, bool force)
  {
    LOCK(lock_);
    if (kinematica_.getEffSize() == 0)
    {
      phi = Eigen::VectorXdRef_ptr();

    }
    else
    {
      if (phi == NULL || force)
      {
        if (phis_.find(task) == phis_.end())
        {
          ERROR("Can't find task '"<<task<<"' in " << object_name_);
          return FAILURE;
        }
        phi = phis_.at(task);
      }
    }
    return SUCCESS;

  }

  EReturn Scene::getJacobian(const std::string & task, Eigen::MatrixXdRef jac)
  {
    LOCK(lock_);
    if (jacs_.find(task) == jacs_.end())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    Eigen::Ref<Eigen::MatrixXd> J(*(jacs_.at(task)));
    for (int r = 0; r < jac.rows(); r++)
    {
      for (int c = 0; c < jac.cols(); c++)
      {
        jac(r, c) = J(r, c);
      }
    }
    return SUCCESS;
  }

  EReturn Scene::getJacobian(const std::string & task,
      Eigen::MatrixXdRef_ptr& jac, bool force)
  {
    LOCK(lock_);
    if (kinematica_.getEffSize() == 0)
    {
      jac = Eigen::MatrixXdRef_ptr();

    }
    else
    {
      if (jac == NULL || force)
      {
        if (jacs_.find(task) == jacs_.end())
        {
          INDICATE_FAILURE
          return FAILURE;
        }
        jac = jacs_.at(task);
      }
    }
    return SUCCESS;
  }

  EReturn Scene::appendTaskMap(const std::string & name,
      const std::vector<std::string> & eff,
      const std::vector<KDL::Frame> & offset)
  {
    LOCK(lock_);
    eff_names_[name] = eff;
    eff_offsets_[name] = offset;

    return SUCCESS;
  }

  EReturn Scene::clearTaskMap()
  {
    eff_names_.clear();
    eff_offsets_.clear();
    return SUCCESS;
  }

  EReturn Scene::getPoses(const std::vector<std::string> & names,
      std::vector<KDL::Frame> & poses)
  {
    LOCK(lock_);
    poses.resize(names.size());
    for (int i = 0; i < names.size(); i++)
    {
      if (!kinematica_.getPose(names[i], poses[i]))
      {
        poses.resize(0);
        INDICATE_FAILURE
        ;
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
      ERROR("Task name: '"<<task<<"'\n"<<eff_index_.size());
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
    exotica::SolutionForm_t tmp_sol;
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
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    std::vector<int> tmp_index;
    if (!kinematica_.getEndEffectorIndex(tmp_index))
    {
      INDICATE_FAILURE
      ;
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
      eff_index_[it.first] = std::vector<int>(tmp_index.begin() + tmp_eff_size,
          tmp_index.begin() + tmp_eff_size + it.second.size());
      phis_[it.first] = Eigen::VectorXdRef_ptr(
          Phi_.segment(tmp_size, 3 * it.second.size()));
      jacs_[it.first] = Eigen::MatrixXdRef_ptr(
          Jac_.block(tmp_size, 0, 3 * it.second.size(), N));
      tmp_size += 3 * it.second.size();
      tmp_eff_size += it.second.size();
    }

    initialised_ = true;

    return SUCCESS;
  }

  EReturn Scene::update(Eigen::VectorXdRefConst x, const int t)
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
        if (kinematica_.getEffSize() > 0)
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
                  INDICATE_FAILURE
                  ;
                  return FAILURE;
                }
              }
              // else Also fine, just skip computing the Jacobians
            }
            else
            {
              INDICATE_FAILURE
              ;
              return FAILURE;
            }
          }
          else
          {
            INDICATE_FAILURE
            ;
            return FAILURE;
          }
        }
      }
      else
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }

    if (visual_debug_->data)
    {
      publishScene();
    }

    return SUCCESS;
  }
  void Scene::publishScene()
  {
    moveit_msgs::PlanningScene msg;
    collision_scene_->getPlanningScene()->getPlanningSceneMsg(msg);
    ps_pub_.publish(msg);
  }

  EReturn Scene::setCollisionScene(
      const planning_scene::PlanningSceneConstPtr & scene)
  {
    moveit_msgs::PlanningScenePtr msg(new moveit_msgs::PlanningScene());
    scene->getPlanningSceneMsg(*msg.get());
    return collision_scene_->initialise(msg, kinematica_.getJointNames(),
        mode_->data, base_type_);
  }

  EReturn Scene::setCollisionScene(
      const moveit_msgs::PlanningSceneConstPtr & scene)
  {
    return collision_scene_->initialise(scene, kinematica_.getJointNames(),
        mode_->data, base_type_);
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

  EReturn Scene::getEndEffectors(const std::string & task,
      std::vector<std::string> & effs)
  {
    if (eff_names_.find(task) == eff_names_.end())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    effs = eff_names_.at(task);
    return SUCCESS;
  }

  EReturn Scene::getEndEffectors(const std::string & task,
      std::pair<std::vector<std::string>, std::vector<KDL::Frame>> & effs)
  {
    if (eff_names_.find(task) == eff_names_.end())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    effs.first = eff_names_.at(task);
    effs.second = eff_offsets_.at(task);
    return SUCCESS;
  }

  int Scene::getMapSize(const std::string & task)
  {
    LOCK(lock_);
    if (eff_names_.find(task) == eff_names_.end()) return -1;
    return eff_names_.at(task).size();
  }

  EReturn Scene::getCoMProperties(std::string& task,
      std::vector<std::string> & segs, Eigen::VectorXd & mass,
      std::vector<KDL::Vector> & cog, std::vector<KDL::Frame> & tip_pose,
      std::vector<KDL::Frame> & base_pose)
  {
    LOCK(lock_);
    if (eff_index_.find(task) == eff_index_.end())
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (kinematica_.getCoMProperties(eff_index_.at(task), segs, mass, cog,
        tip_pose, base_pose))
    {
      return SUCCESS;
    }
    else
    {
      INDICATE_FAILURE
      ;
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

  exotica::KinematicTree & Scene::getSolver()
  {
    return kinematica_;
  }

  EReturn Scene::getJointNames(std::vector<std::string> & joints)
  {
    joints = kinematica_.getJointNames();
    if (joints.size() > 0) return SUCCESS;
    return FAILURE;
  }

  std::string & Scene::getPlanningMode()
  {
    return mode_->data;
  }

  KDL::Frame Scene::getRobotRootWorldTransform()
  {
    return kinematica_.getRobotRootWorldTransform();
  }
}
//	namespace exotica

