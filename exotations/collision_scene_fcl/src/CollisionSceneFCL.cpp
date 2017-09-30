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

#include <collision_scene_fcl/CollisionSceneFCL.h>
#include <exotica/Factory.h>

REGISTER_COLLISION_SCENE_TYPE("CollisionSceneFCL", exotica::CollisionSceneFCL)

namespace fcl_convert
{
fcl::Transform3f KDL2fcl(const KDL::Frame& frame)
{
    return fcl::Transform3f(fcl::Matrix3f(frame.M.data[0], frame.M.data[1], frame.M.data[2], frame.M.data[3], frame.M.data[4], frame.M.data[5], frame.M.data[6], frame.M.data[7], frame.M.data[8]), fcl::Vec3f(frame.p.x(), frame.p.y(), frame.p.z()));
}
}

namespace exotica
{
CollisionSceneFCL::CollisionSceneFCL()
{
}

CollisionSceneFCL::~CollisionSceneFCL()
{
}

void CollisionSceneFCL::updateCollisionObjects(const std::map<std::string, std::shared_ptr<KinematicElement>>& objects)
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

void CollisionSceneFCL::updateCollisionObjectTransforms()
{
    for(fcl::CollisionObject* collision_object : fcl_objects_)
    {
        KinematicElement* element = reinterpret_cast<KinematicElement*>(collision_object->getUserData());
        collision_object->setTransform(fcl_convert::KDL2fcl(element->Frame));
        collision_object->computeAABB();
    }
}

// This function was copied from 'moveit_core/collision_detection_fcl/src/collision_common.cpp'
// https://github.com/ros-planning/moveit/blob/indigo-devel/moveit_core/collision_detection_fcl/src/collision_common.cpp#L512
std::shared_ptr<fcl::CollisionObject> CollisionSceneFCL::constructFclCollisionObject(std::shared_ptr<KinematicElement> element)
{
    // Maybe use cache here?

    shapes::ShapeConstPtr shape = element->Shape;
#ifdef ROS_KINETIC
    std::shared_ptr<fcl::CollisionGeometry> geometry;
#else
    boost::shared_ptr<fcl::CollisionGeometry> geometry;
#endif
    switch (shape->type)
    {
    case shapes::PLANE:
        {
            const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
            geometry.reset(new fcl::Plane(p->a, p->b, p->c, p->d));
        }
        break;
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
    geometry->computeLocalAABB();
    geometry->setUserData(reinterpret_cast<void*>(element.get()));
    std::shared_ptr<fcl::CollisionObject> ret(new fcl::CollisionObject(geometry));
    ret->setUserData(reinterpret_cast<void*>(element.get()));

    return ret;
}

bool CollisionSceneFCL::isAllowedToCollide(fcl::CollisionObject* o1, fcl::CollisionObject* o2, bool self, CollisionSceneFCL* scene)
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

void CollisionSceneFCL::checkCollision(fcl::CollisionObject* o1, fcl::CollisionObject* o2, CollisionData* data)
{
    data->Request.num_max_contacts = 1000;
    data->Result.clear();
    fcl::collide(o1,o2,data->Request, data->Result);
    if(data->SafeDistance>0.0 && o1->getAABB().distance(o2->getAABB())<data->SafeDistance)
    {
        fcl::DistanceRequest req;
        fcl::DistanceResult res;
        req.enable_nearest_points = false;
        fcl::distance(o1, o2, req, res);
        // Add fake contact when distance is smaller than the safety distance.
        if(res.min_distance<data->SafeDistance) data->Result.addContact(fcl::Contact());
    }
}

bool CollisionSceneFCL::collisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data)
{
    CollisionData* data_ = reinterpret_cast<CollisionData*>(data);

    if(!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;

    checkCollision(o1, o2, data_);
    return data_->Result.isCollision();
}

bool CollisionSceneFCL::isStateValid(bool self, double safe_distance)
{
    std::shared_ptr<fcl::BroadPhaseCollisionManager> manager(new fcl::DynamicAABBTreeCollisionManager());
    manager->registerObjects(fcl_objects_);
    CollisionData data(this);
    data.Self = self;
    data.SafeDistance = safe_distance;
    manager->collide(&data, &CollisionSceneFCL::collisionCallback);
    return !data.Result.isCollision();
}

bool CollisionSceneFCL::isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance)
{
    std::vector<fcl::CollisionObject*> shapes1;
    std::vector<fcl::CollisionObject*> shapes2;
    for(fcl::CollisionObject* o : fcl_objects_)
    {
        KinematicElement* e = reinterpret_cast<KinematicElement*>(o->getUserData());
        if(e->Segment.getName()==o1 || e->Parent->Segment.getName()==o1) shapes1.push_back(o);
        if(e->Segment.getName()==o2 || e->Parent->Segment.getName()==o2) shapes2.push_back(o);
    }
    if(shapes1.size()==0) throw_pretty("Can't find object '"<<o1<<"'!");
    if(shapes2.size()==0) throw_pretty("Can't find object '"<<o2<<"'!");
    CollisionData data(this);
    data.SafeDistance = safe_distance;
    for(fcl::CollisionObject* s1 : shapes1)
    {
        for(fcl::CollisionObject* s2 : shapes2)
        {
            checkCollision(s1, s2, &data);
            if(data.Result.isCollision()) return false;
        }
    }
    return true;
}

Eigen::Vector3d CollisionSceneFCL::getTranslation(const std::string & name)
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

std::vector<std::string> CollisionSceneFCL::getCollisionWorldLinks()
{
    std::vector<std::string> tmp;
    for (fcl::CollisionObject* object : fcl_objects_)
    {
        KinematicElement* element = reinterpret_cast<KinematicElement*>(object->getUserData());
        if(!element->ClosestRobotLink)
        {
            tmp.push_back(element->Segment.getName());
        }
    }
    return tmp;
}

/**
   * @brief      Gets the collision robot links.
   *
   * @return     The collision robot links.
   */
std::vector<std::string> CollisionSceneFCL::getCollisionRobotLinks()
{
    std::vector<std::string> tmp;
    for (fcl::CollisionObject* object : fcl_objects_)
    {
        KinematicElement* element = reinterpret_cast<KinematicElement*>(object->getUserData());
        if(element->ClosestRobotLink)
        {
            tmp.push_back(element->Segment.getName());
        }
    }
    return tmp;
}

}
