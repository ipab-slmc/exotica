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
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/octree.h>

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

  fcl::Transform3f KDL2fcl(const KDL::Frame& frame)
  {
      return fcl::Transform3f(fcl::Matrix3f(frame.M.data[0], frame.M.data[1], frame.M.data[2], frame.M.data[3], frame.M.data[4], frame.M.data[5], frame.M.data[6], frame.M.data[7], frame.M.data[8]), fcl::Vec3f(frame.p.x(), frame.p.y(), frame.p.z()));
  }
}
namespace exotica
{

AllowedCollisionMatrix::AllowedCollisionMatrix()
{

}

AllowedCollisionMatrix::AllowedCollisionMatrix(const AllowedCollisionMatrix& acm)
{
    entries_ = acm.entries_;
}

void AllowedCollisionMatrix::clear()
{
    entries_.clear();
}

bool AllowedCollisionMatrix::hasEntry(const std::string& name) const
{
    return entries_.find(name)==entries_.end();
}

void AllowedCollisionMatrix::setEntry(const std::string& name1, const std::string& name2)
{
    entries_[name1].insert(name2);
}

void AllowedCollisionMatrix::getAllEntryNames(std::vector<std::string>& names) const
{
    names.clear();
    for(auto& it : entries_)
    {
        names.push_back(it.first);
    }
}
bool AllowedCollisionMatrix::getAllowedCollision(const std::string& name1, const std::string& name2) const
{
    auto it = entries_.find(name1);
    if(it==entries_.end()) return true;
    return it->second.find(name2)==it->second.end();
}

///////////////////////////////////////////////////////////////
/////////////////////// Collision Scene ///////////////////////
///////////////////////////////////////////////////////////////
  CollisionScene::CollisionScene(const std::string& root_name)
      : root_name_(root_name)
  {
  }

  CollisionScene::~CollisionScene()
  {
  }

  visualization_msgs::Marker CollisionScene::proxyToMarker(const std::vector<CollisionProxy>& proxies)
  {
      visualization_msgs::Marker ret;
      ret.header.frame_id = "exotica/"+root_name_;
      ret.action = visualization_msgs::Marker::ADD;
      ret.frame_locked = false;
      ret.ns = "Proxies";
      ret.color.a=1.0;
      ret.id=0;
      ret.type = visualization_msgs::Marker::LINE_LIST;
      ret.points.resize(proxies.size()*6);
      ret.colors.resize(proxies.size()*6);
      ret.scale.x = 0.005;
      double normalLength = 0.05;
      std_msgs::ColorRGBA normal = getColor(0.8,0.8,0.8);
      std_msgs::ColorRGBA far = getColor(0.5,0.5,0.5);
      std_msgs::ColorRGBA colliding = getColor(1,0,0);
      for(int i=0; i<proxies.size();i++)
      {
          KDL::Vector c1 = proxies[i].e1->Frame*KDL::Vector(proxies[i].contact1(0), proxies[i].contact1(1), proxies[i].contact1(2));
          KDL::Vector c2 = proxies[i].e2->Frame*KDL::Vector(proxies[i].contact2(0), proxies[i].contact2(1), proxies[i].contact2(2));
          KDL::Vector n1 = proxies[i].e1->Frame*KDL::Vector(proxies[i].normal1(0), proxies[i].normal1(1), proxies[i].normal1(2));
          KDL::Vector n2 = proxies[i].e2->Frame*KDL::Vector(proxies[i].normal2(0), proxies[i].normal2(1), proxies[i].normal2(2));
          tf::pointKDLToMsg(c1, ret.points[i*6]);
          tf::pointKDLToMsg(c2, ret.points[i*6+1]);
          tf::pointKDLToMsg(c1, ret.points[i*6+2]);
          tf::pointKDLToMsg(c1+n1*normalLength, ret.points[i*6+3]);
          tf::pointKDLToMsg(c2, ret.points[i*6+4]);
          tf::pointKDLToMsg(c2+n2*normalLength, ret.points[i*6+5]);
          ret.colors[i*6] = proxies[i].distance>0?far:colliding;
          ret.colors[i*6+1] = proxies[i].distance>0?far:colliding;
          ret.colors[i*6+2]=ret.colors[i*6+3]=ret.colors[i*6+4]=ret.colors[i*6+5]=normal;
      }
      return ret;
  }

  void CollisionScene::updateCollisionObjects(const std::map<std::string, std::shared_ptr<KinematicElement>>& objects)
  {
      fcl_objects_.resize(objects.size());
      int i=0;
      for(const auto& object : objects)
      {
          std::shared_ptr<fcl::CollisionObject> new_object;

          const auto& cache_entry = fcl_cache_.find(object.first);
          if(cache_entry == fcl_cache_.end())
          {
              new_object = constructFclCollisionObject(object.second);
              fcl_cache_[object.first] = new_object;
          }
          else
          {
              new_object = cache_entry->second;
          }
          fcl_objects_[i++] = new_object.get();
      }
  }

  void CollisionScene::updateCollisionObjectTransforms()
  {
      for(fcl::CollisionObject* collision_object : fcl_objects_)
      {
          KinematicElement* element = reinterpret_cast<KinematicElement*>(collision_object->getUserData());
          collision_object->setTransform(fcl_convert::KDL2fcl(element->Frame));
          collision_object->computeAABB();
      }
  }

  // This function was copied from 'moveit_core/collision_detection_fcl/src/collision_common.cpp'
  std::shared_ptr<fcl::CollisionObject> CollisionScene::constructFclCollisionObject(std::shared_ptr<KinematicElement> element)
  {
      // Maybe use cache here?

      shapes::ShapeConstPtr shape = element->Shape;
      boost::shared_ptr<fcl::CollisionGeometry> geometry;
      if (shape->type == shapes::PLANE)  // shapes that directly produce CollisionGeometry
      {
        // handle cases individually
        switch (shape->type)
        {
          case shapes::PLANE:
          {
            const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
            geometry.reset(new fcl::Plane(p->a, p->b, p->c, p->d));
          }
          break;
          default:
            break;
        }
      }
      else
      {
        switch (shape->type)
        {
          case shapes::SPHERE:
          {
            const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
            geometry.reset(new fcl::Sphere(s->radius));
          }
          break;
          case shapes::BOX:
          {
            const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
            const double* size = s->size;
            geometry.reset(new fcl::Box(size[0], size[1], size[2]));
          }
          break;
          case shapes::CYLINDER:
          {
            const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
            geometry.reset(new fcl::Cylinder(s->radius, s->length));
          }
          break;
          case shapes::CONE:
          {
            const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
            geometry.reset(new fcl::Cone(s->radius, s->length));
          }
          break;
          case shapes::MESH:
          {
            fcl::BVHModel<fcl::OBBRSS>* g = new fcl::BVHModel<fcl::OBBRSS>();
            const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
            if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
            {
              std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
              for (unsigned int i = 0; i < mesh->triangle_count; ++i)
                tri_indices[i] =
                    fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

              std::vector<fcl::Vec3f> points(mesh->vertex_count);
              for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                points[i] = fcl::Vec3f(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

              g->beginModel();
              g->addSubModel(points, tri_indices);
              g->endModel();
            }
            geometry.reset(g);
          }
          break;
          case shapes::OCTREE:
          {
            const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
            geometry.reset(new fcl::OcTree(g->octree));
          }
          break;
          default:
            throw_pretty("This shape type ("<<((int)shape->type)<<") is not supported using FCL yet");
        }
      }
      geometry->computeLocalAABB();
      geometry->setUserData(reinterpret_cast<void*>(element.get()));
      std::shared_ptr<fcl::CollisionObject> ret(new fcl::CollisionObject(geometry));
      ret->setUserData(reinterpret_cast<void*>(element.get()));

      return ret;
  }

  bool CollisionScene::isAllowedToCollide(fcl::CollisionObject* o1, fcl::CollisionObject* o2, bool self, CollisionScene* scene)
  {
      KinematicElement* e1 = reinterpret_cast<KinematicElement*>(o1->getUserData());
      KinematicElement* e2 = reinterpret_cast<KinematicElement*>(o2->getUserData());

      bool isRobot1 = e1->isRobotLink || e1->ClosestRobotLink;
      bool isRobot2 = e2->isRobotLink || e2->ClosestRobotLink;
      // Don't check collisions between world objects
      if(!isRobot1 && !isRobot2) return false;
      // Skip self collisions if requested
      if(isRobot1 && isRobot2 && !self) return false;
      // Skip collisions between shapes within the same objects
      if(e1->Parent==e2->Parent) return false;
      // Skip collisions between bodies attached to the same object
      if(e1->ClosestRobotLink&&e2->ClosestRobotLink&&e1->ClosestRobotLink==e2->ClosestRobotLink) return false;

      if(isRobot1 && isRobot2)
      {
          const std::string& name1 = e1->ClosestRobotLink?e1->ClosestRobotLink->Segment.getName():e1->Parent->Segment.getName();
          const std::string& name2 = e2->ClosestRobotLink?e2->ClosestRobotLink->Segment.getName():e2->Parent->Segment.getName();
          return scene->acm_.getAllowedCollision(name1, name2);
      }
      return true;
  }

  bool CollisionScene::collisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data)
  {
      CollisionData* data_ = reinterpret_cast<CollisionData*>(data);

      if(!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;

      data_->Request.num_max_contacts = 1000;
      data_->Result.clear();
      fcl::collide(o1,o2,data_->Request, data_->Result);
      data_->Done = data_->Result.isCollision();
      return data_->Done;
  }

  bool CollisionScene::collisionCallbackDistance(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& dist)
  {
      DistanceData* data_ = reinterpret_cast<DistanceData*>(data);

      if(!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;
      data_->Request.enable_nearest_points = false;
      data_->Result.clear();
      fcl::distance(o1,o2,data_->Request, data_->Result);
      int num_contacts = data_->Distance = std::min(data_->Distance, data_->Result.min_distance);

      CollisionProxy p;
      p.e1 = reinterpret_cast<KinematicElement*>(o1->getUserData());
      p.e2 = reinterpret_cast<KinematicElement*>(o2->getUserData());

      p.distance = data_->Result.min_distance;
      fcl_convert::fcl2Eigen(data_->Result.nearest_points[0], p.contact1);
      fcl_convert::fcl2Eigen(data_->Result.nearest_points[1], p.contact2);
      p.normal1 = p.contact1.normalized();
      p.normal2 = p.contact2.normalized();
      p.distance = (p.contact1-p.contact2).norm();
      data_->Distance = std::min(data_->Distance, p.distance);
      data_->Proxies.push_back(p);

      return false;
  }

  bool CollisionScene::collisionCallbackContacts(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& dist)
  {
      ContactData* data_ = reinterpret_cast<ContactData*>(data);

      if(!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;

      data_->Request.enable_contact = true;
      data_->Request.num_max_contacts = 1000;
      data_->Result.clear();
      int num_contacts = fcl::collide(o1, o2, data_->Request, data_->Result);
      CollisionProxy p;
      p.e1 = reinterpret_cast<KinematicElement*>(o1->getUserData());
      p.e2 = reinterpret_cast<KinematicElement*>(o2->getUserData());

      if(num_contacts>0)
      {
          p.distance = -data_->Result.getContact(0).penetration_depth;
          for(int i=0; i<num_contacts; i++)
          {
              const fcl::Contact& contact = data_->Result.getContact(i);
              if(p.distance>-contact.penetration_depth)
              {
                  p.distance=-contact.penetration_depth;
                  if(reinterpret_cast<KinematicElement*>(contact.o1->getUserData())==p.e1)
                  {
                      fcl_convert::fcl2Eigen(contact.pos, p.contact1);
                      fcl_convert::fcl2Eigen(contact.normal, p.normal1);
                      p.normal1.normalize();
                  }
                  else if(reinterpret_cast<KinematicElement*>(contact.o2->getUserData())==p.e2)
                  {
                      fcl_convert::fcl2Eigen(contact.pos, p.contact2);
                      fcl_convert::fcl2Eigen(contact.normal, p.normal2);
                      p.normal2.normalize();
                  }
              }
          }
      }
      else
      {
          data_->DistanceRequest.enable_nearest_points = true;
          data_->DistanceRequest.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
          fcl::distance(o1,o2,data_->DistanceRequest, data_->DistanceResult);
          fcl_convert::fcl2Eigen(data_->DistanceResult.nearest_points[0], p.contact1);
          fcl_convert::fcl2Eigen(data_->DistanceResult.nearest_points[1], p.contact2);
          p.normal1 = p.contact1.normalized();
          p.normal2 = p.contact2.normalized();
          p.distance = (p.contact1-p.contact2).norm();
      }
      data_->Distance = std::min(data_->Distance, p.distance);
      data_->Proxies.push_back(p);

      return false;
  }

  bool CollisionScene::isStateValid(bool self)
  {
      std::shared_ptr<fcl::BroadPhaseCollisionManager> manager(new fcl::DynamicAABBTreeCollisionManager());
      manager->registerObjects(fcl_objects_);
      CollisionData data(this);
      data.Self = self;
      manager->collide(&data, &CollisionScene::collisionCallback);
      return !data.Result.isCollision();
  }

  std::vector<CollisionProxy> CollisionScene::getCollisionDistance(bool self, bool computePenetrationDepth)
  {
      std::shared_ptr<fcl::BroadPhaseCollisionManager> manager(new fcl::DynamicAABBTreeCollisionManager());
      manager->registerObjects(fcl_objects_);
      if(computePenetrationDepth)
      {
          ContactData data(this);
          data.Self = self;
          manager->distance(&data, &CollisionScene::collisionCallbackContacts);
          return data.Proxies;
      }
      else
      {
          DistanceData data(this);
          data.Self = self;
          manager->distance(&data, &CollisionScene::collisionCallbackDistance);
          return data.Proxies;
      }
  }


  Eigen::Vector3d CollisionScene::getTranslation(const std::string & name)
  {
      for(fcl::CollisionObject* object : fcl_objects_)
      {
          KinematicElement* element = reinterpret_cast<KinematicElement*>(object->getUserData());
          if(element->Segment.getName()==name)
          {
              return Eigen::Map<Eigen::Vector3d>(element->Frame.p.data);
          }
      }
      throw_pretty("Robot not found!");;
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

      collision_scene_.reset(new CollisionScene(kinematica_.getRootFrameName()));
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
          proxy_pub_.publish(collision_scene_->proxyToMarker(proxies));
      }
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

