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
#include <iostream>
#include <fstream>
#include <string>

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

    if (model_->getSRDF()->getVirtualJoints().size() > 0)
      world_joint_ = model_->getSRDF()->getVirtualJoints()[0].name_;

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

    if (compute_dist) {
      reinitializeCollisionRobot();
      reinitializeCollisionWorld();
    }
  }

  void CollisionScene::reinitializeCollisionRobot() {
    // Reinitialize robot
    fcl_robot_.clear();
    geo_robot_.clear();
    ps_->getCurrentStateNonConst().update(true);
    const std::vector<const robot_model::LinkModel*>& links =
        ps_->getCollisionRobot()
            ->getRobotModel()
            ->getLinkModelsWithCollisionGeometry();
    for (std::size_t i = 0; i < links.size(); ++i) {
      geo_robot_[links[i]->getName()] = geos_ptr(0);
      fcl_robot_[links[i]->getName()] = fcls_ptr(0);
      for (std::size_t j = 0; j < links[i]->getShapes().size(); ++j) {
        shapes::ShapeConstPtr tmp_shape;
        if (links[i]->getShapes()[j]->type != shapes::MESH)
          tmp_shape = shapes::ShapeConstPtr(
              shapes::createMeshFromShape(links[i]->getShapes()[j].get()));
        else
          tmp_shape = links[i]->getShapes()[j];
        if (!tmp_shape || !tmp_shape.get())
          throw_pretty("Shape could not be extracted");

        collision_detection::FCLGeometryConstPtr g =
            collision_detection::createCollisionGeometry(tmp_shape, links[i],
                                                         j);
        if (g) {
          geo_robot_.at(links[i]->getName()).push_back(g);
          fcl::CollisionObject* tmp = new fcl::CollisionObject(
              g->collision_geometry_,
              collision_detection::transform2fcl(
                  ps_->getCurrentState().getCollisionBodyTransform(
                      g->collision_geometry_data_->ptr.link,
                      g->collision_geometry_data_->shape_index)));
          fcl_robot_.at(links[i]->getName())
              .push_back(std::shared_ptr<fcl::CollisionObject>(tmp));

        } else
          throw_pretty("Unable to construct collision geometry for link "
                       << links[i]->getName().c_str());
      }
    }
  }

  void CollisionScene::reinitializeCollisionWorld() {
    // Reinitialize world
    fcl_world_.clear();
    geo_world_.clear();
    collision_detection::WorldConstPtr tmp_world =
        ps_->getCollisionWorld()->getWorld();
    std::vector<std::string> obj_id_ = tmp_world->getObjectIds();
    if (obj_id_.size() > 0) {
      for (std::size_t i = 0; i < obj_id_.size(); ++i) {
        std::size_t index_size =
            tmp_world->getObject(obj_id_[i])->shapes_.size();
        fcl_world_[obj_id_[i]] = fcls_ptr(0);
        geo_world_[obj_id_[i]] = geos_ptr(0);
        trans_world_[obj_id_[i]] = std::vector<fcl::Transform3f>(0);
        for (std::size_t j = 0; j < index_size; j++) {
          shapes::ShapeConstPtr tmp_shape;

          if (tmp_world->getObject(obj_id_[i])->shapes_[j]->type ==
              shapes::OCTREE) {
            tmp_world->getObject(obj_id_[i])->shapes_[j]->print();
            tmp_shape = boost::static_pointer_cast<const shapes::Shape>(
                tmp_world->getObject(obj_id_[i])->shapes_[j]);
          } else {
            if (tmp_world->getObject(obj_id_[i])->shapes_[j]->type !=
                shapes::MESH) {
              tmp_shape = shapes::ShapeConstPtr(shapes::createMeshFromShape(
                  tmp_world->getObject(obj_id_[i])->shapes_[j].get()));
            } else {
              tmp_shape = tmp_world->getObject(obj_id_[i])->shapes_[j];
            }
          }

          if (!tmp_shape || !tmp_shape.get())
            throw_pretty("Could not extract shape");

          collision_detection::FCLGeometryConstPtr g =
              collision_detection::createCollisionGeometry(
                  tmp_shape, tmp_world->getObject(obj_id_[i]).get());
          geo_world_.at(obj_id_[i]).push_back(g);
          trans_world_.at(obj_id_[i])
              .push_back(fcl::Transform3f(collision_detection::transform2fcl(
                  tmp_world->getObject(obj_id_[i])->shape_poses_[j])));
          fcl_world_.at(obj_id_[i])
              .push_back(std::shared_ptr<fcl::CollisionObject>(
                  new fcl::CollisionObject(
                      g->collision_geometry_,
                      collision_detection::transform2fcl(
                          tmp_world->getObject(obj_id_[i])->shape_poses_[j]))));
        }
      }
    }
  }

  void CollisionScene::updateCollisionRobot() {
    for (auto& it : fcl_robot_) {
      for (std::size_t i = 0; i < it.second.size(); ++i) {
        collision_detection::CollisionGeometryData* cd =
            static_cast<collision_detection::CollisionGeometryData*>(
                it.second[i]->collisionGeometry()->getUserData());
        it.second[i]->setTransform(collision_detection::transform2fcl(
            ps_->getCurrentState().getCollisionBodyTransform(cd->ptr.link,
                                                             cd->shape_index)));
        it.second[i]->getTransform().transform(
            it.second[i]->collisionGeometry()->aabb_center);
      }
    }
  }

  void CollisionScene::updateWorld(
      const moveit_msgs::PlanningSceneWorldConstPtr& world) {
    ps_->processPlanningSceneWorldMsg(*world);

    if (compute_dist)
      reinitializeCollisionWorld();
  }

  void CollisionScene::update(std::string joint, double value) {
    try {
      ps_->getCurrentStateNonConst().setVariablePosition(joint, value);
    } catch (const std::exception& ex) {
      throw_pretty("Exception while trying to update individual joint '"
                   << joint << "': " << ex.what());
    }
    ps_->getCurrentStateNonConst().update(true);

    // Update FCL robot link
    if (compute_dist)
      updateCollisionRobot();
  }

  void CollisionScene::update(Eigen::VectorXdRefConst x)
  {
    if (joint_index_.size() != x.rows())
      throw_pretty("Size does not match, need vector size of "
                   << joint_index_.size() << " but " << x.rows()
                   << " is provided");

    if (BaseType == BASE_TYPE::FIXED)
    {
      for (std::size_t i = 0; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    else if (BaseType == BASE_TYPE::FLOATING)
    {
      ps_->getCurrentStateNonConst().setVariablePosition(
          world_joint_ + "/trans_x", x(0));
      ps_->getCurrentStateNonConst().setVariablePosition(
          world_joint_ + "/trans_y", x(1));
      ps_->getCurrentStateNonConst().setVariablePosition(
          world_joint_ + "/trans_z", x(2));
      KDL::Rotation rot = KDL::Rotation::EulerZYX(x(3), x(4), x(5));
      Eigen::VectorXd quat(4);
      rot.GetQuaternion(quat(0), quat(1), quat(2), quat(3));
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/rot_x",
          quat(0));
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/rot_y",
          quat(1));
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/rot_z",
          quat(2));
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/rot_w",
          quat(3));
      for (std::size_t i = 6; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    else if (BaseType == BASE_TYPE::PLANAR)
    {
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/x",
          x(0));
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/y",
          x(1));
      ps_->getCurrentStateNonConst().setVariablePosition(world_joint_ + "/theta",
          x(2));
      for (std::size_t i = 3; i < joint_index_.size(); i++)
        ps_->getCurrentStateNonConst().setVariablePosition(joint_index_[i],
            x(i));
    }
    ps_->getCurrentStateNonConst().update(true);

    // Update FCL robot state for collision distance computation
    if (compute_dist)
      updateCollisionRobot();
  }

  // Public overload returning only the distance
  void CollisionScene::getDistance(const std::string& o1, const std::string& o2,
                                   double& d, double safeDist) {
    Eigen::Vector3d p1, p2;
    getDistance(o1, o2, d, false, safeDist, p1, p2);
  }

  // Public overload computing the nearest points
  void CollisionScene::getDistance(const std::string& o1, const std::string& o2,
                                   double& d, Eigen::Vector3d& p1,
                                   Eigen::Vector3d& p2, double safeDist) {
    getDistance(o1, o2, d, true, safeDist, p1, p2);
  }

  // Private method to private functionality, wrapped via public overloads
  void CollisionScene::getDistance(const std::string& o1, const std::string& o2,
                                   double& d,
                                   const bool calculateContactInformation,
                                   const double safeDist, Eigen::Vector3d& p1,
                                   Eigen::Vector3d& p2) {
    fcls_ptr fcl1, fcl2;
    if (fcl_robot_.find(o1) != fcl_robot_.end())
      fcl1 = fcl_robot_.at(o1);
    else if (fcl_world_.find(o1) != fcl_world_.end())
      fcl1 = fcl_world_.at(o1);
    else
      throw_pretty("Object 1 not found!");

    if (fcl_world_.find(o2) != fcl_world_.end())
      fcl2 = fcl_world_.at(o2);
    else if (fcl_robot_.find(o2) != fcl_robot_.end())
      fcl2 = fcl_robot_.at(o2);
    else
      throw_pretty("Object 2 not found!");

    fcl::DistanceRequest req;
    req.enable_nearest_points = calculateContactInformation;
    fcl::DistanceResult res;
    d = distance(fcl1, fcl2, req, res, safeDist);

    // distance() returns either a distance > 0 or -1 if in collision
    if (d > 0) {
      d = res.min_distance;

      if (calculateContactInformation) {
        fcl_convert::fcl2Eigen(res.nearest_points[0], p1);
        fcl_convert::fcl2Eigen(res.nearest_points[1], p2);
      }
    } else if (d == -1) {  // If d == -1, we need to compute the contact to get
                           // a penetration depth
      fcl::CollisionRequest c_req;
      c_req.enable_contact = true;
      c_req.num_max_contacts = 500;
      fcl::CollisionResult c_res;
      computeContact(fcl1, fcl2, c_req, c_res, d, p1, p2);
      d *= -1;
      // ROS_INFO_STREAM("Penetration depth: "
      //                 << d << " at world position=" << pos.transpose()
      //                 << " with contact normal=" << norm.transpose());
    } else {
      throw_pretty("We should never end up here. Why?");
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
        if (fcl1[i] == nullptr) throw_pretty("Object 1 not found!");

        if (fcl2[j] == nullptr) throw_pretty("Object 2 not found!");

        if (fcl2[j]->getAABB().distance(fcl2[j]->getAABB()) < safeDist)
        {
          if (fcl::distance(fcl1[i].get(), fcl2[j].get(), req, tmp) < 0)
          {
            throw_pretty(
                "If this condition is triggered something has changed about "
                "FCL's distance computation as this was not working in 0.3.4 "
                "(Trusty). Need to reconsider logic - please open an issue on "
                "GitHub.");

            res = tmp;
            res.min_distance = -1;
            return -1;
          }
          else
          {
            // If the current closest distance is less than previous, update the
            // DistanceResult object
            if (tmp.min_distance < res.min_distance) res = tmp;

            // The distance request returns 0 i.e. the two FCL objects are in
            // contact. Now need to do a CollisionRequest in order to obtain the
            // penetration depth and contact normals. However, we will return
            // -1 here and have another method penetrationDepth carry out
            // these computations.
            if (res.min_distance == 0.0) return -1;
          }

          tmp.clear();
        }
      }
    }

    return res.min_distance;
  }

  void CollisionScene::computeContact(
      const fcls_ptr& fcl1, const fcls_ptr& fcl2,
      const fcl::CollisionRequest& req, fcl::CollisionResult& res,
      double& penetration_depth, Eigen::Vector3d& pos, Eigen::Vector3d& norm) {
    fcl::CollisionResult tmp;
    penetration_depth = 0;
    for (int i = 0; i < fcl1.size(); i++) {
      for (int j = 0; j < fcl2.size(); j++) {
        if (fcl1[i] == nullptr) throw_pretty("Object 1 not found!");
        if (fcl2[j] == nullptr) throw_pretty("Object 2 not found!");

        std::size_t num_contacts =
            fcl::collide(fcl1[i].get(), fcl2[j].get(), req, tmp);

        if (num_contacts == 0) {
          throw_pretty("Objects are not in contact.");
        } else {
          ROS_INFO_STREAM("Objects have " << num_contacts
                                          << " contact points.");

          // Iterate over contacts and compare maximum penetration
          for (std::size_t k = 0; k < num_contacts; k++) {
            auto& contact = tmp.getContact(k);
            // ROS_INFO_STREAM("Contact #" << k << " has depth of " << contact.penetration_depth);
            if (contact.penetration_depth > penetration_depth) {
              penetration_depth = contact.penetration_depth;
              fcl_convert::fcl2Eigen(contact.normal, norm);
              fcl_convert::fcl2Eigen(contact.pos, pos);
              norm = norm.normalized();
              res = tmp;
              // ROS_INFO_STREAM("New deepest penetration depth: " << penetration_depth);
            }
          }
        }
        tmp.clear();
      }
    }
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

      collision_scene_.reset(new CollisionScene(name_));

      BaseType = kinematica_.getControlledBaseType();

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
          collision_scene_->initialise(msg, kinematica_.getJointNames(), "", BaseType, model_);
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
    updateSceneFrames();
  }

  void Scene::setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene)
  {
    collision_scene_->initialise(scene, kinematica_.getJointNames(), "", BaseType,model_);
    updateSceneFrames();
  }

  CollisionScene_ptr & Scene::getCollisionScene()
  {
    return collision_scene_;
  }

  std::string Scene::getRootFrameName()
  {
    return kinematica_.getRootFrameName();
  }

  std::string Scene::getRootJointName()
  {
    return kinematica_.getRootJointName();
  }

  std::string Scene::getModelRootLinkName()
  {
    return model_->getRootLinkName();
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
    // Update Kinematica internal state
    kinematica_.setModelState(x);

    // Update Planning Scene State
    int i = 0;
    for (auto& joint : getModelJointNames())
    {
      collision_scene_->update(joint, x(i));
      i++;
    }

    if (debug_) publishScene();
  }

  void Scene::setModelState(std::map<std::string, double> x) {
    // Update Kinematica internal state
    kinematica_.setModelState(x);

    // Update Planning Scene State
    for (auto& joint : x)
      collision_scene_->update(joint.first, joint.second);

    if (debug_) publishScene();
  }

  Eigen::VectorXd Scene::getControlledState()
  {
      return kinematica_.getControlledState();
  }

  std::string Scene::getGroupName()
  {
      return group->getName();
  }

  void Scene::loadScene(const std::string& scene)
  {
      std::stringstream ss(scene);
      getPlanningScene()->loadGeometryFromStream(ss);
      updateSceneFrames();
      collision_scene_->reinitializeCollisionWorld();
  }

  void Scene::loadSceneFile(const std::string& file_name)
  {
      std::ifstream ss(parsePath(file_name));
      getPlanningScene()->loadGeometryFromStream(ss);
      updateSceneFrames();
      collision_scene_->reinitializeCollisionWorld();
  }

  std::string Scene::getScene()
  {
      std::stringstream ss;
      getPlanningScene()->saveGeometryToStream(ss);
      return ss.str();
  }

  void Scene::cleanScene()
  {
      getPlanningScene()->removeAllCollisionObjects();
      updateSceneFrames();
      collision_scene_->reinitializeCollisionWorld();
  }

  void Scene::updateSceneFrames()
  {
      kinematica_.resetModel();

      planning_scene::PlanningScenePtr ps = collision_scene_->getPlanningScene();

      // Add world objects
      for(auto& object : *ps->getWorld())
      {
          if(object.second->shapes_.size())
          {
              // Use the first collision shape as the origin of the object
              Eigen::Affine3d objTransform = object.second->shape_poses_[0];
              kinematica_.AddElement(object.first, objTransform);
              for(int i=0; i<object.second->shape_poses_.size();i++)
              {
                  Eigen::Affine3d trans = objTransform.inverse()*object.second->shape_poses_[i];
                  kinematica_.AddElement(object.first+"_collision_"+std::to_string(i), trans, object.first, object.second->shapes_[i]);
              }
          }
          else
          {
            HIGHLIGHT("Object with no shapes ('"<<object.first<<"')");
          }
      }

      // Add robot collision objects
      ps->getCurrentStateNonConst().update(true);
      const std::vector<const robot_model::LinkModel*>& links =
          ps->getCollisionRobot()->getRobotModel()->getLinkModelsWithCollisionGeometry();
      for (int i = 0; i < links.size(); ++i)
      {
          Eigen::Affine3d objTransform = ps->getCurrentState().getGlobalLinkTransform(links[i]);

          for (int j = 0; j < links[i]->getShapes().size(); ++j)
          {
              Eigen::Affine3d trans = objTransform.inverse()*ps->getCurrentState().getCollisionBodyTransform(links[i], j);
              kinematica_.AddElement(links[i]->getName()+"_collision_"+std::to_string(j), trans, links[i]->getName(), links[i]->getShapes()[j]);
          }
      }

      kinematica_.UpdateModel();
  }

}
//  namespace exotica

