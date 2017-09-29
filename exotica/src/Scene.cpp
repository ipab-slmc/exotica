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
#include <exotica/Setup.h>
#include <iostream>
#include <fstream>
#include <string>

namespace exotica
{
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
      kinematica_.Instantiate(init.JointGroup, model_, name_);
      group = model_->getJointModelGroup(init.JointGroup);
      ps_.reset(new planning_scene::PlanningScene(model_));

      BaseType = kinematica_.getControlledBaseType();

      if (Server::isRos()) {
        ps_pub_ = Server::advertise<moveit_msgs::PlanningScene>(name_ +(name_==""?"":"/")+"PlanningScene", 100, true);
        proxy_pub_ = Server::advertise<visualization_msgs::Marker>(name_ +(name_==""?"":"/")+"CollisionProxies", 100, true);
        if (debug_)
          HIGHLIGHT_NAMED(
              name_,
              "Running in debug mode, planning scene will be published to '"
                  << Server::Instance()->getName() << "/" << name_
                  << "/PlanningScene'");
      }

      collision_scene_ = Setup::createCollisionScene(init.CollisionScene);
      collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());

      AllowedCollisionMatrix acm;
      std::vector<std::string> acm_names;
      ps_->getAllowedCollisionMatrix().getAllEntryNames(acm_names);
      for(auto& name1 : acm_names)
      {
          for(auto& name2 : acm_names)
          {
              collision_detection::AllowedCollision::Type type = collision_detection::AllowedCollision::Type::ALWAYS;
              ps_->getAllowedCollisionMatrix().getAllowedCollision(name1, name2, type);
              if(type == collision_detection::AllowedCollision::Type::ALWAYS)
              {
                  acm.setEntry(name1, name2);
              }
          }
      }
      collision_scene_->setACM(acm);

      if (debug_) INFO_NAMED(name_, "Exotica Scene initialized");
  }

  std::shared_ptr<KinematicResponse> Scene::RequestKinematics(KinematicsRequest& Request)
  {
      return kinematica_.RequestFrames(Request);
  }

  void Scene::Update(Eigen::VectorXdRefConst x)
  {
      kinematica_.Update(x);
      collision_scene_->updateCollisionObjectTransforms();
      if (debug_) publishScene();
  }

  void Scene::publishScene()
  {
    if(Server::isRos())
    {
        moveit_msgs::PlanningScene msg;
        ps_->getPlanningSceneMsg(msg);
        ps_pub_.publish(msg);
    }
  }

  void Scene::publishProxies(const std::vector<CollisionProxy>& proxies)
  {
      if(Server::isRos())
      {
          proxy_pub_.publish(proxyToMarker(proxies, kinematica_.getRootFrameName()));
      }
  }

  visualization_msgs::Marker Scene::proxyToMarker(const std::vector<CollisionProxy>& proxies, const std::string& frame)
  {
      visualization_msgs::Marker ret;
      ret.header.frame_id = "exotica/"+frame;
      ret.action = visualization_msgs::Marker::ADD;
      ret.frame_locked = false;
      ret.ns = "Proxies";
      ret.color.a=1.0;
      ret.id=0;
      ret.type = visualization_msgs::Marker::LINE_LIST;
      ret.points.resize(proxies.size()*6);
      ret.colors.resize(proxies.size()*6);
      ret.scale.x = 0.005;
      double normalLength = 0.01;
      std_msgs::ColorRGBA normal = getColor(0.8,0.8,0.8);
      std_msgs::ColorRGBA far = getColor(0.5,0.5,0.5);
      std_msgs::ColorRGBA colliding = getColor(1,0,0);
      for(int i=0; i<proxies.size();i++)
      {
          KDL::Vector c1 = KDL::Vector(proxies[i].contact1(0), proxies[i].contact1(1), proxies[i].contact1(2));
          KDL::Vector c2 = KDL::Vector(proxies[i].contact2(0), proxies[i].contact2(1), proxies[i].contact2(2));
          KDL::Vector n1 = KDL::Vector(proxies[i].normal1(0), proxies[i].normal1(1), proxies[i].normal1(2));
          KDL::Vector n2 = KDL::Vector(proxies[i].normal2(0), proxies[i].normal2(1), proxies[i].normal2(2));
          tf::pointKDLToMsg(c1, ret.points[i*6]);
          tf::pointKDLToMsg(c2, ret.points[i*6+1]);
          tf::pointKDLToMsg(c1, ret.points[i*6+2]);
          tf::pointKDLToMsg(c1+n1*normalLength, ret.points[i*6+3]);
          tf::pointKDLToMsg(c2, ret.points[i*6+4]);
          tf::pointKDLToMsg(c2+n2*normalLength, ret.points[i*6+5]);
          ret.colors[i*6] = ret.colors[i*6+1] = proxies[i].distance>0?far:colliding;
          ret.colors[i*6+2]=ret.colors[i*6+3]=ret.colors[i*6+4]=ret.colors[i*6+5]=normal;
      }
      return ret;
  }

  void Scene::setCollisionScene(const moveit_msgs::PlanningSceneConstPtr & scene)
  {
    updateSceneFrames();
    collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());
  }

  void Scene::updateWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world)
  {
      updateSceneFrames();
      collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());
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
    return ps_;
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

    collision_scene_->updateCollisionObjectTransforms();

    if (debug_) publishScene();
  }

  void Scene::setModelState(std::map<std::string, double> x) {
    // Update Kinematica internal state
    kinematica_.setModelState(x);

    collision_scene_->updateCollisionObjectTransforms();

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
      ps_->loadGeometryFromStream(ss);
      updateSceneFrames();
      collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());
  }

  void Scene::loadSceneFile(const std::string& file_name)
  {
      std::ifstream ss(parsePath(file_name));
      ps_->loadGeometryFromStream(ss);
      updateSceneFrames();
      collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());
  }

  std::string Scene::getScene()
  {
      std::stringstream ss;
      ps_->saveGeometryToStream(ss);
      return ss.str();
  }

  void Scene::cleanScene()
  {
      ps_->removeAllCollisionObjects();
      updateSceneFrames();
      collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());
  }

  void Scene::updateSceneFrames()
  {
      kinematica_.resetModel();

      // Add world objects
      for(auto& object : *ps_->getWorld())
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
      ps_->getCurrentStateNonConst().update(true);
      const std::vector<const robot_model::LinkModel*>& links =
          ps_->getCollisionRobot()->getRobotModel()->getLinkModelsWithCollisionGeometry();
      for (int i = 0; i < links.size(); ++i)
      {
          Eigen::Affine3d objTransform = ps_->getCurrentState().getGlobalLinkTransform(links[i]);

          for (int j = 0; j < links[i]->getShapes().size(); ++j)
          {
              Eigen::Affine3d trans = objTransform.inverse()*ps_->getCurrentState().getCollisionBodyTransform(links[i], j);
              kinematica_.AddElement(links[i]->getName()+"_collision_"+std::to_string(j), trans, links[i]->getName(), links[i]->getShapes()[j]);
          }
      }

      kinematica_.UpdateModel();
  }

  void Scene::attachObject(const std::string& name, const std::string& parent)
  {
      kinematica_.changeParent(name, parent, KDL::Frame(), false);
      attached_objects_[name] = AttachedObject(parent);
  }

  void Scene::attachObjectLocal(const std::string& name, const std::string& parent, const KDL::Frame& pose)
  {
      kinematica_.changeParent(name, parent, pose, true);
      attached_objects_[name] = AttachedObject(parent, pose);
  }

  void Scene::detachObject(const std::string& name)
  {
      if(!hasAttachedObject(name)) throw_pretty("'"<<name<<"' is not attached to the robot!");
      auto object = attached_objects_.find(name);
      kinematica_.changeParent(name, "", KDL::Frame(), false);
      attached_objects_.erase(object);
  }

  bool Scene::hasAttachedObject(const std::string& name)
  {
      return attached_objects_.find(name)!=attached_objects_.end();
  }

}
//  namespace exotica

