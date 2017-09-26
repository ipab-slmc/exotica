/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#include <collision_scene_fcl_latest/CollisionSceneFCLLatest.h>
#include <exotica/Factory.h>
#include <eigen_conversions/eigen_kdl.h>

REGISTER_COLLISION_SCENE_TYPE("CollisionSceneFCLLatest", exotica::CollisionSceneFCLLatest)

namespace fcl_convert
{
void fcl2Eigen(const fcl::Vector3d & fcl, Eigen::Vector3d & eigen)
{
    eigen(0) = fcl(0);
    eigen(1) = fcl(1);
    eigen(2) = fcl(2);
}

void fcl2Eigen(const fcl::Transform3d & fcl, Eigen::Vector3d & eigen)
{
    eigen(0) = fcl.translation()(0);
    eigen(1) = fcl.translation()(1);
    eigen(2) = fcl.translation()(2);
}

void fcl2EigenTranslation(const fcl::Vector3d & fcl, Eigen::Vector3d & eigen)
{
    eigen(0) = fcl(0);
    eigen(1) = fcl(1);
    eigen(2) = fcl(2);
}

fcl::Transform3d KDL2fcl(const KDL::Frame& frame)
{
    Eigen::Affine3d ret;
    tf::transformKDLToEigen(frame, ret);
    return fcl::Transform3d(ret);
}
}

namespace exotica
{
CollisionSceneFCLLatest::CollisionSceneFCLLatest()
{
    HIGHLIGHT("FCL version: "<<FCL_VERSION);
}

CollisionSceneFCLLatest::~CollisionSceneFCLLatest()
{
}

void CollisionSceneFCLLatest::updateCollisionObjects(const std::map<std::string, std::shared_ptr<KinematicElement>>& objects)
{
    fcl_objects_.resize(objects.size());
    int i=0;
    for(const auto& object : objects)
    {
        std::shared_ptr<fcl::CollisionObjectd> new_object;

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

void CollisionSceneFCLLatest::updateCollisionObjectTransforms()
{
    for(fcl::CollisionObjectd* collision_object : fcl_objects_)
    {
        KinematicElement* element = reinterpret_cast<KinematicElement*>(collision_object->getUserData());
        collision_object->setTransform(fcl_convert::KDL2fcl(element->Frame));
        collision_object->computeAABB();
    }
}

// This function was copied from 'moveit_core/collision_detection_fcl/src/collision_common.cpp'
std::shared_ptr<fcl::CollisionObjectd> CollisionSceneFCLLatest::constructFclCollisionObject(std::shared_ptr<KinematicElement> element)
{
    // Maybe use cache here?

    shapes::ShapeConstPtr shape = element->Shape;
    std::shared_ptr<fcl::CollisionGeometryd> geometry;
    if (shape->type == shapes::PLANE)  // shapes that directly produce CollisionGeometry
    {
        // handle cases individually
        switch (shape->type)
        {
        case shapes::PLANE:
        {
            const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
            geometry.reset(new fcl::Planed(p->a, p->b, p->c, p->d));
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
            geometry.reset(new fcl::Sphered(s->radius));
        }
            break;
        case shapes::BOX:
        {
            const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
            const double* size = s->size;
            geometry.reset(new fcl::Boxd(size[0], size[1], size[2]));
        }
            break;
        case shapes::CYLINDER:
        {
            const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
            geometry.reset(new fcl::Cylinderd(s->radius, s->length));
        }
            break;
        case shapes::CONE:
        {
            const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
            geometry.reset(new fcl::Coned(s->radius, s->length));
        }
            break;
        case shapes::MESH:
        {
            fcl::BVHModel<fcl::OBBRSSd>* g = new fcl::BVHModel<fcl::OBBRSSd>();
            const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
            if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
            {
                std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
                for (unsigned int i = 0; i < mesh->triangle_count; ++i)
                    tri_indices[i] =
                            fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

                std::vector<fcl::Vector3d> points(mesh->vertex_count);
                for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                    points[i] = fcl::Vector3d(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

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
            geometry.reset(new fcl::OcTreed(to_std_ptr(g->octree)));
        }
            break;
        default:
            throw_pretty("This shape type ("<<((int)shape->type)<<") is not supported using FCL yet");
        }
    }
    geometry->computeLocalAABB();
    geometry->setUserData(reinterpret_cast<void*>(element.get()));
    std::shared_ptr<fcl::CollisionObjectd> ret(new fcl::CollisionObjectd(geometry));
    ret->setUserData(reinterpret_cast<void*>(element.get()));

    return ret;
}

bool CollisionSceneFCLLatest::isAllowedToCollide(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, bool self, CollisionSceneFCLLatest* scene)
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

bool CollisionSceneFCLLatest::collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
    CollisionData* data_ = reinterpret_cast<CollisionData*>(data);

    if(!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;

    data_->Request.num_max_contacts = 1000;
    data_->Result.clear();
    fcl::collide(o1, o2, data_->Request, data_->Result);
    data_->Done = data_->Result.isCollision();
    return data_->Done;
}

bool CollisionSceneFCLLatest::collisionCallbackDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& dist)
{
    DistanceData* data_ = reinterpret_cast<DistanceData*>(data);

    if(!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;
    data_->Request.enable_nearest_points = true;
    data_->Request.enable_signed_distance = true;
    // GST_LIBCCD produces incorrect contacts. Probably due to incompatible version of libccd.
    data_->Request.gjk_solver_type = fcl::GST_INDEP;
    data_->Result.clear();
    fcl::distance(o1,o2,data_->Request, data_->Result);

    CollisionProxy p;
    p.e1 = reinterpret_cast<KinematicElement*>(o1->getUserData());
    p.e2 = reinterpret_cast<KinematicElement*>(o2->getUserData());

    p.distance = data_->Result.min_distance;
    if(p.distance>0)
    {
        KDL::Vector c1 = p.e1->Frame*KDL::Vector(data_->Result.nearest_points[0](0), data_->Result.nearest_points[0](1), data_->Result.nearest_points[0](2));
        KDL::Vector c2 = p.e2->Frame*KDL::Vector(data_->Result.nearest_points[1](0), data_->Result.nearest_points[1](1), data_->Result.nearest_points[1](2));
        KDL::Vector n1 = c2-c1;
        KDL::Vector n2 = c1-c2;
        n1.Normalize();
        n2.Normalize();
        tf::vectorKDLToEigen(c1, p.contact1);
        tf::vectorKDLToEigen(c2, p.contact2);
        tf::vectorKDLToEigen(n1, p.normal1);
        tf::vectorKDLToEigen(n2, p.normal2);
    }
    else
    {
        // Contact points are not reliable (FCL bug), use shape centres instead.
        KDL::Vector c1 = p.e1->Frame.p;
        KDL::Vector c2 = p.e2->Frame.p;
        KDL::Vector n1 = c2-c1;
        KDL::Vector n2 = c1-c2;
        n1.Normalize();
        n2.Normalize();
        tf::vectorKDLToEigen(c1, p.contact1);
        tf::vectorKDLToEigen(c2, p.contact2);
        tf::vectorKDLToEigen(n1, p.normal1);
        tf::vectorKDLToEigen(n2, p.normal2);
    }
    data_->Distance = std::min(data_->Distance, p.distance);
    data_->Proxies.push_back(p);
    return false;
}

bool CollisionSceneFCLLatest::isStateValid(bool self)
{
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager(new fcl::DynamicAABBTreeCollisionManagerd());
    manager->registerObjects(fcl_objects_);
    CollisionData data(this);
    data.Self = self;
    manager->collide(&data, &CollisionSceneFCLLatest::collisionCallback);
    return !data.Result.isCollision();
}

std::vector<CollisionProxy> CollisionSceneFCLLatest::getCollisionDistance(bool self, bool computePenetrationDepth)
{
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager(new fcl::DynamicAABBTreeCollisionManagerd());
    manager->registerObjects(fcl_objects_);
    DistanceData data(this);
    data.Self = self;
    manager->distance(&data, &CollisionSceneFCLLatest::collisionCallbackDistance);
    return data.Proxies;
}

Eigen::Vector3d CollisionSceneFCLLatest::getTranslation(const std::string & name)
{
    for(fcl::CollisionObjectd* object : fcl_objects_)
    {
        KinematicElement* element = reinterpret_cast<KinematicElement*>(object->getUserData());
        if(element->Segment.getName()==name)
        {
            return Eigen::Map<Eigen::Vector3d>(element->Frame.p.data);
        }
    }
    throw_pretty("Robot not found!");;
}
}
