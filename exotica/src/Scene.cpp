/*
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
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
/////////////////////// Collision Scene ///////////////////////
///////////////////////////////////////////////////////////////
  CollisionScene::CollisionScene(const std::string & scene_name)
      : compute_dist(true), stateCheckCnt_(0), scene_name_(scene_name)
  {
  }

  CollisionScene::~CollisionScene()
  {
    //TODO
  }

  void CollisionScene::initialise(
      const moveit_msgs::PlanningSceneConstPtr & msg,
      const std::vector<std::string> & joints, const std::string & mode,
      BASE_TYPE base_type, robot_model::RobotModelPtr model_)
  {
    ps_.reset(new planning_scene::PlanningScene(model_));

    if (!acm_)
    {
      acm_.reset(
          new collision_detection::AllowedCollisionMatrix(
              ps_->getAllowedCollisionMatrix()));
    }
    ps_->setPlanningSceneMsg(*msg.get());
    BaseType = base_type;
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
  }

  void CollisionScene::updateWorld(
      const moveit_msgs::PlanningSceneWorldConstPtr& world) {
    ps_->processPlanningSceneWorldMsg(*world);

    //    for (int i=0;i<world->collision_objects.size();i++)
    //    {
    //      if(trans_world_.find(world->collision_objects[i].id) !=
    //      trans_world_.end())
    //      {
    //        std::map<std::string, fcls_ptr>::iterator it =
    //        trans_world_.find(world->collision_objects[i].id);
    //        for(int j =0;j<it->second.size();j++)
    //        {
    //          it->second[j] =
    //          fcl::Transform3f(collision_detection::transform2fcl(world->collision_objects[i].mesh_poses[j]));
    //        }
    //      }
    //    }
  }

  void CollisionScene::update(Eigen::VectorXdRefConst x)
  {
    if (joint_index_.size() != x.rows())
    {
      throw_pretty(
          "Size does not match, need vector size of "<<joint_index_.size()<<" but "<<x.rows()<<" is provided");
    }

    if (BaseType == BASE_TYPE::FIXED)
    {
      for (std::size_t i = 0; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    else if (BaseType == BASE_TYPE::FLOATING)
    {
      const std::string world_name =
          ps_->getCurrentStateNonConst().getRobotModel()->getSRDF()->getVirtualJoints()[0].name_;
      ps_->getCurrentStateNonConst().setVariablePosition(
          world_name + "/trans_x", x(0));
      ps_->getCurrentStateNonConst().setVariablePosition(
          world_name + "/trans_y", x(1));
      ps_->getCurrentStateNonConst().setVariablePosition(
          world_name + "/trans_z", x(2));
      KDL::Rotation rot = KDL::Rotation::EulerZYX(x(3), x(4), x(5));
      Eigen::VectorXd quat(4);
      rot.GetQuaternion(quat(0), quat(1), quat(2), quat(3));
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/rot_x",
          quat(0));
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/rot_y",
          quat(1));
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/rot_z",
          quat(2));
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/rot_w",
          quat(3));
      for (std::size_t i = 6; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    else if (BaseType == BASE_TYPE::PLANAR)
    {
      const std::string world_name =
          ps_->getCurrentStateNonConst().getRobotModel()->getSRDF()->getVirtualJoints()[0].name_;
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/x",
          x(0));
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/y",
          x(1));
      ps_->getCurrentStateNonConst().setVariablePosition(world_name + "/theta",
          x(2));
      for (std::size_t i = 3; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
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
  }

  void CollisionScene::getDistance(const std::string & o1,
      const std::string & o2, double& d, double safeDist)
  {
    fcls_ptr fcl1, fcl2;
    if (fcl_robot_.find(o1) != fcl_robot_.end())
      fcl1 = fcl_robot_.at(o1);
    else if (fcl_world_.find(o1) != fcl_world_.end())
      fcl1 = fcl_world_.at(o1);
    else
    {
      throw_pretty("Object 1 not found!");
    }
    if (fcl_world_.find(o2) != fcl_world_.end())
      fcl2 = fcl_world_.at(o2);
    else if (fcl_robot_.find(o2) != fcl_robot_.end())
      fcl2 = fcl_robot_.at(o2);
    else
    {
      throw_pretty("Object 2 not found!");
    }

    fcl::DistanceRequest req(false);
    fcl::DistanceResult res;
    d = distance(fcl1, fcl2, req, res, safeDist);
  }

  void CollisionScene::getDistance(const std::string & o1,
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
      throw_pretty("Object 1 not found!");
    }
    if (fcl_world_.find(o2) != fcl_world_.end())
      fcl2 = fcl_world_.at(o2);
    else if (fcl_robot_.find(o2) != fcl_robot_.end())
      fcl2 = fcl_robot_.at(o2);
    else
    {
      throw_pretty("Object 2 not found!");
    }

    fcl::DistanceRequest req(true);
    fcl::DistanceResult res;
    if (distance(fcl1, fcl2, req, res, safeDist) >= 0)
    {
      d = res.min_distance;
      fcl_convert::fcl2Eigen(res.nearest_points[0], p1);
      fcl_convert::fcl2Eigen(res.nearest_points[1], p2);
    }
  }

  bool CollisionScene::isStateValid(bool self, double dist)
  {
    stateCheckCnt_++;
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    if (self)
    {
      ps_->checkSelfCollision(req, res, ps_->getCurrentState(), *acm_);
      if (res.collision)
      {
        return false;
      }
    }
    req.contacts = false;
    if (dist > 0) req.distance = true;
    ps_->getCollisionWorld()->checkRobotCollision(req, res,
        *ps_->getCollisionRobot(), ps_->getCurrentState());
    return dist == 0 ? !res.collision : res.distance > dist;
  }

  // NB: need to check whether this function works as intended
  bool CollisionScene::isStateValid(const Eigen::VectorXd &q)
  {
    update(q);
    return ps_->isStateValid(ps_->getCurrentState());
  }

  double CollisionScene::getClosestDistance() {
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;

    double selfCollisionDistance, distanceToEnvironment;

    // Check for both self- and world-collisions
    req.contacts = false;
    req.distance = true;
    ps_->checkCollision(req, res, getCurrentState(), *acm_);

    if (res.collision)
      ROS_ERROR_STREAM("State in collision: " << res.distance);
    else
      ROS_INFO_STREAM("State NOT in collision: " << res.distance);

    return res.distance;
  }

  void CollisionScene::getRobotDistance(const std::string & link, bool self,
      double & d, Eigen::Vector3d & p1, Eigen::Vector3d & p2,
      Eigen::Vector3d & norm, Eigen::Vector3d & c1, Eigen::Vector3d & c2,
      double safeDist)
  {
    fcls_ptr fcl_link;
    if (fcl_robot_.find(link) != fcl_robot_.end())
      fcl_link = fcl_robot_.at(link);
    else
    {
      throw_pretty("Link not found!");
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
        if (link.compare(it.first) != 0 && acm_->getEntry(link, it.first, type))
        {
          if (type == collision_detection::AllowedCollision::NEVER)
          {
            ROS_INFO_STREAM_THROTTLE(2,
                "Checking between "<<link<<" and "<<it.first);
            for (std::size_t i = 0; i < it.second.size(); i++)
            {
              if (distance(fcl_link, it.second, req, res, safeDist) < 0)
              {
//              INDICATE_FAILURE
                d = -1;
                return; // WARNING;
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
          return;
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

    norm = p2 - p1;
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
          throw_pretty("Object 1 not found!");
        }
        if (fcl2[j] == nullptr)
        {
          throw_pretty("Object 2 not found!");
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

  void CollisionScene::getCollisionLinkTranslation(const std::string & name,
      Eigen::Vector3d & translation)
  {
    if (fcl_robot_.find(name) == fcl_robot_.end()) throw_pretty("Robot not found!");;
    std::map<std::string, fcls_ptr>::iterator it = fcl_robot_.find(name);
    fcl::AABB sumAABB;
    for (int i = 0; i < it->second.size(); i++)
    {
      it->second[i]->computeAABB();
      sumAABB += it->second[i]->getAABB();
    }
    fcl_convert::fcl2Eigen(sumAABB.center(), translation);
  }

  void CollisionScene::getWorldObjectTranslation(const std::string & name,
      Eigen::Vector3d & translation)
  {
    if (fcl_world_.find(name) == fcl_world_.end()) throw_pretty("Robot not found!");;
    std::map<std::string, fcls_ptr>::iterator it = fcl_world_.find(name);
    fcl::AABB sumAABB;
    for (int i = 0; i < it->second.size(); i++)
    {
      it->second[i]->computeAABB();
      sumAABB += it->second[i]->getAABB();
    }
    fcl_convert::fcl2Eigen(sumAABB.center(), translation);
  }

  void CollisionScene::getTranslation(const std::string & name,
      Eigen::Vector3d & translation)
  {
    getCollisionLinkTranslation(name, translation);
    getWorldObjectTranslation(name, translation);
  }

///////////////////////////////////////////////////////////////
/////////////////////// EXOTica Scene ///////////////////////
///////////////////////////////////////////////////////////////

  Scene::Scene() : name_("Unnamed")
  {

  }

  Scene::Scene(const std::string & name) : name_(name)
  {
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

  void Scene::Instantiate(SceneInitializer& init)
  {
      Object::InstatiateObject(init);
      name_ = object_name_;
      kinematica_.Debug = debug_;
      if(init.URDF=="" || init.SRDF=="")
      {
          Server::Instance()->getModel(init.RobotDescription, model_);
      }
      else
      {
          Server::Instance()->getModel(init.URDF, model_, init.URDF, init.SRDF);
      }
      kinematica_.Instantiate(init.JointGroup, model_);
      group = model_->getJointModelGroup(init.JointGroup);

      BaseType = kinematica_.getBaseType();

      collision_scene_.reset(new CollisionScene(name_));

      if (Server::isRos()) {
        ps_pub_ = Server::advertise<moveit_msgs::PlanningScene>(
            name_ + "/PlanningScene", 100, true);
        if (debug_)
          HIGHLIGHT_NAMED(
              name_,
              "Running in debug mode, planning scene will be published to '"
                  << Server::Instance()->getName() << "/" << name_
                  << "/PlanningScene'");
      }

      {
          planning_scene::PlanningScenePtr tmp(new planning_scene::PlanningScene(model_));
          moveit_msgs::PlanningScenePtr msg(new moveit_msgs::PlanningScene());
          tmp->getPlanningSceneMsg(*msg.get());
          collision_scene_->initialise(msg, kinematica_.getJointNames(), "", BaseType,model_);
      }
      
      if (debug_) INFO_NAMED(name_, "Exotica Scene initialized");
  }

  std::shared_ptr<KinematicResponse> Scene::RequestKinematics(KinematicsRequest& Request)
  {
      return kinematica_.RequestFrames(Request);
  }

  void Scene::Update(Eigen::VectorXdRefConst x)
  {
      collision_scene_->update(x);
      kinematica_.Update(x);
      if (debug_) publishScene();
  }

  void Scene::publishScene()
  {
    if(Server::isRos())
    {
        moveit_msgs::PlanningScene msg;
        collision_scene_->getPlanningScene()->getPlanningSceneMsg(msg);
        ps_pub_.publish(msg);
    }
  }

  void Scene::setCollisionScene(
      const planning_scene::PlanningSceneConstPtr & scene)
  {
    moveit_msgs::PlanningScenePtr msg(new moveit_msgs::PlanningScene());
    scene->getPlanningSceneMsg(*msg.get());
    collision_scene_->initialise(msg, kinematica_.getJointNames(), "", BaseType, model_);
  }

  void Scene::setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene)
  {
    collision_scene_->initialise(scene, kinematica_.getJointNames(), "", BaseType,model_);
  }

  int Scene::getNumJoints()
  {
    return kinematica_.getNumJoints();
  }

  CollisionScene_ptr & Scene::getCollisionScene()
  {
    return collision_scene_;
  }

  std::string Scene::getRootName()
  {
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

  void Scene::getJointNames(std::vector<std::string> & joints)
  {
    joints = kinematica_.getJointNames();
  }

  std::vector<std::string> Scene::getJointNames()
  {
    return kinematica_.getJointNames();
  }

  std::vector<std::string> Scene::getModelJointNames()
  {
    return kinematica_.getModelJointNames();
  }

  Eigen::VectorXd Scene::getModelState()
  {
    return kinematica_.getModelState();
  }

  std::map<std::string, double> Scene::getModelStateMap()
  {
    return kinematica_.getModelStateMap();
  }

  void Scene::setModelState(Eigen::VectorXdRefConst x)
  {
    kinematica_.setModelState(x);
  }

  void Scene::setModelState(std::map<std::string, double> x)
  {
    kinematica_.setModelState(x);
  }

  Eigen::VectorXd Scene::getControlledState()
  {
      return kinematica_.getControlledState();
  }

}
//  namespace exotica

