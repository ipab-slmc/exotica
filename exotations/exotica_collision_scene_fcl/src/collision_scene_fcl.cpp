//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_collision_scene_fcl/collision_scene_fcl.h>
#include <exotica_core/factory.h>

#include <ros/ros.h>

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
CollisionSceneFCL::CollisionSceneFCL() = default;
CollisionSceneFCL::~CollisionSceneFCL() = default;

void CollisionSceneFCL::Setup()
{
    if (debug_) HIGHLIGHT_NAMED("CollisionSceneFCL", "FCL version: " << FCL_VERSION);
}

void CollisionSceneFCL::UpdateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects)
{
    kinematic_elements_ = MapToVec(objects);
    fcl_cache_.clear();
    fcl_objects_.resize(objects.size());
    long i = 0;
    for (const auto& object : objects)
    {
        std::shared_ptr<fcl::CollisionObject> new_object;

        // const auto& cache_entry = fcl_cache_.find(object.first);
        // TODO: There is currently a bug with the caching causing proxies not
        // to update. The correct fix would be to update the user data, for now
        // disable use of the cache.
        // if (true)  // (cache_entry == fcl_cache_.end())
        //{
        new_object = ConstructFclCollisionObject(i, object.second.lock());
        fcl_cache_[object.first] = new_object;
        //}
        // else
        // {
        //     new_object = cache_entry->second;
        // }
        fcl_objects_[i++] = new_object.get();
    }
}

void CollisionSceneFCL::UpdateCollisionObjectTransforms()
{
    for (fcl::CollisionObject* collision_object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(collision_object->getUserData())].lock();
        if (!element)
        {
            ThrowPretty("Expired pointer, this should not happen - make sure to call UpdateCollisionObjects() after UpdateSceneFrames()");
        }
        collision_object->setTransform(fcl_convert::KDL2fcl(element->frame));
        collision_object->computeAABB();
    }
}

// This function was copied from 'moveit_core/collision_detection_fcl/src/collision_common.cpp'
// https://github.com/ros-planning/moveit/blob/indigo-devel/moveit_core/collision_detection_fcl/src/collision_common.cpp#L512
std::shared_ptr<fcl::CollisionObject> CollisionSceneFCL::ConstructFclCollisionObject(long kinematic_element_id, std::shared_ptr<KinematicElement> element)
{
    shapes::ShapeConstPtr shape = element->shape;

    // Apply scaling and padding
    if (element->is_robot_link || element->closest_robot_link.lock())
    {
        if (robot_link_scale_ != 1.0 || robot_link_padding_ > 0.0)
        {
            shapes::ShapePtr scaled_shape(shape->clone());
            scaled_shape->scaleAndPadd(robot_link_scale_, robot_link_padding_);
            shape = scaled_shape;
        }
    }
    else
    {
        if (world_link_scale_ != 1.0 || world_link_padding_ > 0.0)
        {
            shapes::ShapePtr scaled_shape(shape->clone());
            scaled_shape->scaleAndPadd(world_link_scale_, world_link_padding_);
            shape = scaled_shape;
        }
    }

// Uses `std::shared_ptr` if ROS version >= ROS_KINETIC
#if ROS_VERSION_MINIMUM(1, 12, 0)
    std::shared_ptr<fcl::CollisionGeometry> geometry;
#else
    boost::shared_ptr<fcl::CollisionGeometry> geometry;
#endif
    switch (shape->type)
    {
        case shapes::PLANE:
        {
            auto p = dynamic_cast<const shapes::Plane*>(shape.get());
            geometry.reset(new fcl::Plane(p->a, p->b, p->c, p->d));
        }
        break;
        case shapes::SPHERE:
        {
            auto s = dynamic_cast<const shapes::Sphere*>(shape.get());
            geometry.reset(new fcl::Sphere(s->radius));
        }
        break;
        case shapes::BOX:
        {
            auto s = dynamic_cast<const shapes::Box*>(shape.get());
            const double* size = s->size;
            geometry.reset(new fcl::Box(size[0], size[1], size[2]));
        }
        break;
        case shapes::CYLINDER:
        {
            auto s = dynamic_cast<const shapes::Cylinder*>(shape.get());
            bool degenerate_capsule = (s->length <= 2 * s->radius);
            if (!replace_cylinders_with_capsules || degenerate_capsule)
            {
                geometry.reset(new fcl::Cylinder(s->radius, s->length));
            }
            else
            {
                geometry.reset(new fcl::Capsule(s->radius, s->length - 2 * s->radius));
            }
        }
        break;
        case shapes::CONE:
        {
            auto s = dynamic_cast<const shapes::Cone*>(shape.get());
            geometry.reset(new fcl::Cone(s->radius, s->length));
        }
        break;
        case shapes::MESH:
        {
            auto g = new fcl::BVHModel<fcl::OBBRSS>();
            auto mesh = dynamic_cast<const shapes::Mesh*>(shape.get());
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
            auto g = dynamic_cast<const shapes::OcTree*>(shape.get());
            geometry.reset(new fcl::OcTree(g->octree));
        }
        break;
        default:
            ThrowPretty("This shape type (" << ((int)shape->type) << ") is not supported using FCL yet");
    }
    geometry->computeLocalAABB();
    geometry->setUserData(reinterpret_cast<void*>(kinematic_element_id));
    std::shared_ptr<fcl::CollisionObject> ret(new fcl::CollisionObject(geometry));
    ret->setUserData(reinterpret_cast<void*>(kinematic_element_id));

    return ret;
}

bool CollisionSceneFCL::IsAllowedToCollide(fcl::CollisionObject* o1, fcl::CollisionObject* o2, bool self, CollisionSceneFCL* scene)
{
    std::shared_ptr<KinematicElement> e1 = scene->kinematic_elements_[reinterpret_cast<long>(o1->getUserData())].lock();
    std::shared_ptr<KinematicElement> e2 = scene->kinematic_elements_[reinterpret_cast<long>(o2->getUserData())].lock();

    bool isRobot1 = e1->is_robot_link || e1->closest_robot_link.lock();
    bool isRobot2 = e2->is_robot_link || e2->closest_robot_link.lock();
    // Don't check collisions between world objects
    if (!isRobot1 && !isRobot2) return false;
    // Skip self collisions if requested
    if (isRobot1 && isRobot2 && !self) return false;
    // Skip collisions between shapes within the same objects
    if (e1->parent.lock() == e2->parent.lock()) return false;
    // Skip collisions between bodies attached to the same object
    if (e1->closest_robot_link.lock() && e2->closest_robot_link.lock() && e1->closest_robot_link.lock() == e2->closest_robot_link.lock()) return false;

    if (isRobot1 && isRobot2)
    {
        const std::string& name1 = e1->closest_robot_link.lock() ? e1->closest_robot_link.lock()->segment.getName() : e1->parent.lock()->segment.getName();
        const std::string& name2 = e2->closest_robot_link.lock() ? e2->closest_robot_link.lock()->segment.getName() : e2->parent.lock()->segment.getName();
        return scene->acm_.getAllowedCollision(name1, name2);
    }
    return true;
}

void CollisionSceneFCL::CheckCollision(fcl::CollisionObject* o1, fcl::CollisionObject* o2, CollisionData* data)
{
    data->request.num_max_contacts = 1000;
    data->result.clear();
    fcl::collide(o1, o2, data->request, data->result);
    if (data->safe_distance > 0.0 && o1->getAABB().distance(o2->getAABB()) < data->safe_distance)
    {
        fcl::DistanceRequest req;
        fcl::DistanceResult res;
        req.enable_nearest_points = false;
        fcl::distance(o1, o2, req, res);
        // Add fake contact when distance is smaller than the safety distance.
        if (res.min_distance < data->safe_distance) data->result.addContact(fcl::Contact());
    }
}

bool CollisionSceneFCL::CollisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data)
{
    CollisionData* data_ = reinterpret_cast<CollisionData*>(data);

    if (!IsAllowedToCollide(o1, o2, data_->self, data_->scene)) return false;

    CheckCollision(o1, o2, data_);
    return data_->result.isCollision();
}

bool CollisionSceneFCL::IsStateValid(bool self, double safe_distance)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::shared_ptr<fcl::BroadPhaseCollisionManager> manager(new fcl::DynamicAABBTreeCollisionManager());
    manager->registerObjects(fcl_objects_);
    CollisionData data(this);
    data.self = self;
    data.safe_distance = safe_distance;
    manager->collide(&data, &CollisionSceneFCL::CollisionCallback);
    return !data.result.isCollision();
}

bool CollisionSceneFCL::IsCollisionFree(const std::string& o1, const std::string& o2, double safe_distance)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::vector<fcl::CollisionObject*> shapes1;
    std::vector<fcl::CollisionObject*> shapes2;
    for (fcl::CollisionObject* o : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())].lock();
        if (e->segment.getName() == o1 || e->parent.lock()->segment.getName() == o1) shapes1.push_back(o);
        if (e->segment.getName() == o2 || e->parent.lock()->segment.getName() == o2) shapes2.push_back(o);
    }
    if (shapes1.size() == 0) ThrowPretty("Can't find object '" << o1 << "'!");
    if (shapes2.size() == 0) ThrowPretty("Can't find object '" << o2 << "'!");
    CollisionData data(this);
    data.safe_distance = safe_distance;
    for (fcl::CollisionObject* s1 : shapes1)
    {
        for (fcl::CollisionObject* s2 : shapes2)
        {
            CheckCollision(s1, s2, &data);
            if (data.result.isCollision()) return false;
        }
    }
    return true;
}

Eigen::Vector3d CollisionSceneFCL::GetTranslation(const std::string& name)
{
    for (fcl::CollisionObject* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (element->segment.getName() == name)
        {
            return Eigen::Map<Eigen::Vector3d>(element->frame.p.data);
        }
    }
    ThrowPretty("Robot not found!");
}

std::vector<std::string> CollisionSceneFCL::GetCollisionWorldLinks()
{
    std::vector<std::string> tmp;
    for (fcl::CollisionObject* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (!element->closest_robot_link.lock())
        {
            tmp.push_back(element->segment.getName());
        }
    }
    return tmp;
}

std::vector<std::shared_ptr<KinematicElement>> CollisionSceneFCL::GetCollisionWorldLinkElements()
{
    std::vector<std::shared_ptr<KinematicElement>> tmp;
    for (fcl::CollisionObject* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (!element->closest_robot_link.lock())
        {
            tmp.push_back(element);
        }
    }
    return tmp;
}

std::vector<std::string> CollisionSceneFCL::GetCollisionRobotLinks()
{
    std::vector<std::string> tmp;
    for (fcl::CollisionObject* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (element->closest_robot_link.lock())
        {
            tmp.push_back(element->segment.getName());
        }
    }
    return tmp;
}
}
