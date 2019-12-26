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

#include <exotica_collision_scene_fcl_latest/collision_scene_fcl_latest.h>
#include <exotica_core/factory.h>

REGISTER_COLLISION_SCENE_TYPE("CollisionSceneFCLLatest", exotica::CollisionSceneFCLLatest)

#define CONTINUOUS_COLLISION_USE_ADVANCED_SETTINGS
// #define CONTINUOUS_COLLISION_DEBUG

namespace fcl_convert
{
// ** Do not remove the inline ** It causes an epic memory bug in the transforms.
//      Fixed in July 2018 and July 2019.
inline fcl::Transform3d KDL2fcl(const KDL::Frame& frame)
{
    Eigen::Isometry3d ret;
    tf::transformKDLToEigen(frame, ret);
    return ret;
}
}

namespace exotica
{
CollisionSceneFCLLatest::CollisionSceneFCLLatest() = default;
CollisionSceneFCLLatest::~CollisionSceneFCLLatest() = default;

void CollisionSceneFCLLatest::Setup()
{
    if (debug_) HIGHLIGHT_NAMED("CollisionSceneFCLLatest", "FCL version: " << FCL_VERSION);
}

void CollisionSceneFCLLatest::UpdateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects)
{
    kinematic_elements_.clear();
    fcl_cache_.clear();
    fcl_objects_.clear();
    fcl_objects_.reserve(objects.size());
    long i = 0;
    for (const auto& object : objects)
    {
        // Check whether object is excluded as a world collision object:
        if (world_links_to_exclude_from_collision_scene.count(object.first) > 0)
        {
            if (debug_) HIGHLIGHT_NAMED("CollisionSceneFCLLatest::UpdateCollisionObject", object.first << " is excluded, skipping.");
        }
        else
        {
            if (debug_) HIGHLIGHT_NAMED("CollisionSceneFCLLatest::UpdateCollisionObject", "Creating " << object.first);
            std::shared_ptr<fcl::CollisionObjectd> new_object;

            // const auto& cache_entry = fcl_cache_.find(object.first);
            // TODO: There is currently a bug with the caching causing proxies not
            // to update. The correct fix would be to update the user data, for now
            // disable use of the cache.
            // if (true)  // (cache_entry == fcl_cache_.end())
            // {
            new_object = ConstructFclCollisionObject(i, object.second.lock());
            fcl_cache_[object.first] = new_object;
            // }
            // else
            // {
            //     new_object = cache_entry->second;
            // }
            fcl_objects_.emplace_back(new_object.get());
            kinematic_elements_.emplace_back(object.second);
            ++i;
        }
    }
}

void CollisionSceneFCLLatest::UpdateCollisionObjectTransforms()
{
    for (fcl::CollisionObjectd* collision_object : fcl_objects_)
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

// This function was originally copied from 'moveit_core/collision_detection_fcl/src/collision_common.cpp'
// https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_core/collision_detection_fcl/src/collision_common.cpp#L520
// and then modified for use in EXOTica.
std::shared_ptr<fcl::CollisionObjectd> CollisionSceneFCLLatest::ConstructFclCollisionObject(long kinematic_element_id, std::shared_ptr<KinematicElement> element)
{
    shapes::ShapePtr shape(element->shape->clone());

    // Apply scaling and padding
    if (element->is_robot_link || element->closest_robot_link.lock())
    {
        if (robot_link_scale_ != 1.0 || robot_link_padding_ > 0.0)
        {
            shape->scaleAndPadd(robot_link_scale_, robot_link_padding_);
        }
    }
    else
    {
        if (world_link_scale_ != 1.0 || world_link_padding_ > 0.0)
        {
            shape->scaleAndPadd(world_link_scale_, world_link_padding_);
        }
    }

    // Replace primitive shapes with meshes if desired (e.g. if primitives are unstable)
    if (replace_primitive_shapes_with_meshes_)
    {
        if (static_cast<int>(shape->type) < 6)  // The regular enum type comparisons start to fail at times :/
        {
            shape.reset(reinterpret_cast<shapes::Shape*>(shapes::createMeshFromShape(shape.get())));
        }
    }

    std::shared_ptr<fcl::CollisionGeometryd> geometry;
    switch (shape->type)
    {
        case shapes::PLANE:
        {
            auto p = dynamic_cast<const shapes::Plane*>(shape.get());
            geometry.reset(new fcl::Planed(p->a, p->b, p->c, p->d));
        }
        break;
        case shapes::SPHERE:
        {
            auto s = dynamic_cast<const shapes::Sphere*>(shape.get());
            geometry.reset(new fcl::Sphered(s->radius));
        }
        break;
        case shapes::BOX:
        {
            auto s = dynamic_cast<const shapes::Box*>(shape.get());
            const double* size = s->size;
            geometry.reset(new fcl::Boxd(size[0], size[1], size[2]));
        }
        break;
        case shapes::CYLINDER:
        {
            auto s = dynamic_cast<const shapes::Cylinder*>(shape.get());
            bool degenerate_capsule = (s->length <= 2 * s->radius);
            if (!replace_cylinders_with_capsules || degenerate_capsule)
            {
                geometry.reset(new fcl::Cylinderd(s->radius, s->length));
            }
            else
            {
                geometry.reset(new fcl::Capsuled(s->radius, s->length - 2 * s->radius));
            }
        }
        break;
        case shapes::CONE:
        {
            auto s = dynamic_cast<const shapes::Cone*>(shape.get());
            geometry.reset(new fcl::Coned(s->radius, s->length));
        }
        break;
        case shapes::MESH:
        {
            auto g = new fcl::BVHModel<fcl::OBBRSSd>();
            auto mesh = dynamic_cast<const shapes::Mesh*>(shape.get());
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
            auto g = dynamic_cast<const shapes::OcTree*>(shape.get());
            geometry.reset(new fcl::OcTreed(ToStdPtr(g->octree)));
        }
        break;
        default:
            ThrowPretty("This shape type (" << ((int)shape->type) << ") is not supported using FCL yet");
    }
    geometry->computeLocalAABB();
    geometry->setUserData(reinterpret_cast<void*>(kinematic_element_id));
    std::shared_ptr<fcl::CollisionObjectd> ret(new fcl::CollisionObjectd(geometry));
    ret->setUserData(reinterpret_cast<void*>(kinematic_element_id));

    return ret;
}

bool CollisionSceneFCLLatest::IsAllowedToCollide(const std::string& o1, const std::string& o2, const bool& self)
{
    std::shared_ptr<KinematicElement> e1, e2;

    for (auto kinematic_element : kinematic_elements_)
    {
        std::shared_ptr<KinematicElement> tmp = kinematic_element.lock();
        if (tmp->segment.getName() == o1) e1 = tmp;
        if (tmp->segment.getName() == o2) e2 = tmp;
    }

    if (!e1) ThrowPretty("o1 is not a valid collision link:" << o1);
    if (!e2) ThrowPretty("o2 is not a valid collision link:" << o2);

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
        return acm_.getAllowedCollision(name1, name2);
    }
    return true;
}

bool CollisionSceneFCLLatest::IsAllowedToCollide(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, bool self, CollisionSceneFCLLatest* scene)
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

void CollisionSceneFCLLatest::CheckCollision(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, CollisionData* data)
{
    data->request.num_max_contacts = 1000;
    data->request.gjk_solver_type = fcl::GST_LIBCCD;
    data->result.clear();
    fcl::collide(o1, o2, data->request, data->result);
    if (data->safe_distance > 0.0 && o1->getAABB().distance(o2->getAABB()) < data->safe_distance)
    {
        fcl::DistanceRequestd req;
        fcl::DistanceResultd res;
        req.enable_nearest_points = false;
        fcl::distance(o1, o2, req, res);
        // Add fake contact when distance is smaller than the safety distance.
        if (res.min_distance < data->safe_distance) data->result.addContact(fcl::Contactd());
    }
}

bool CollisionSceneFCLLatest::CollisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
    CollisionData* data_ = reinterpret_cast<CollisionData*>(data);

    if (!IsAllowedToCollide(o1, o2, data_->self, data_->scene)) return false;

    CheckCollision(o1, o2, data_);
    return data_->result.isCollision();
}

void CollisionSceneFCLLatest::ComputeDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, DistanceData* data)
{
    // Setup proxy.
    CollisionProxy p;
    p.e1 = data->scene->kinematic_elements_[reinterpret_cast<long>(o1->getUserData())].lock();
    p.e2 = data->scene->kinematic_elements_[reinterpret_cast<long>(o2->getUserData())].lock();

    // New logic as of August 2018:
    //  - Use LIBCCD as comment in Drake suggests it is more reliable now.
    //  - If in collision, use the deepest contact of the collide callback.
    //  - If not in collision, run distance query.

    // Step 0: Run collision check:
    fcl::CollisionRequestd tmp_req;
    fcl::CollisionResultd tmp_res;
    tmp_req.num_max_contacts = 1000;
    tmp_req.enable_contact = true;

    // The following comment and Step 1 code is copied from Drake (BSD license):
    // https://github.com/RobotLocomotion/drake/blob/0aa7f713eb029fea7d47109992762ed6d8d1d457/geometry/proximity_engine.cc
    // NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
    // superior to FCL's "independent" implementation. Furthermore, libccd
    // appears to behave badly if its gjk tolerance is much tighter than
    // 2e-12. Until this changes, we explicitly specify these parameters rather
    // than relying on FCL's defaults.
    tmp_req.gjk_tolerance = 2e-12;
    tmp_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    fcl::collide(o1, o2, tmp_req, tmp_res);

    // Step 1: If in collision, extract contact point.
    if (tmp_res.isCollision())
    {
        // TODO: Issue #364: https://github.com/ipab-slmc/exotica/issues/364
        // As of 0.5.94, this does not work for primitive-vs-mesh (but does for mesh-vs-primitive):
        if ((o1->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM && o2->getObjectType() == fcl::OBJECT_TYPE::OT_BVH) || (o1->getObjectType() == fcl::OBJECT_TYPE::OT_BVH && o2->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM))  // || (o1->getObjectType() == fcl::OBJECT_TYPE::OT_BVH && o2->getObjectType() == fcl::OBJECT_TYPE::OT_BVH))
        {
            // WARNING("As of 0.5.94, this function does not work for primitive-vs-mesh and vice versa. Do not expect the contact points or distances to be accurate at all.");
        }

        // Some of the logic below is copied from Drake
        std::vector<fcl::Contactd> contacts;
        tmp_res.getContacts(contacts);
        if (!contacts.empty())
        {
            size_t deepest_penetration_depth_index = -1;
            double deepest_penetration_depth = -1;
            for (size_t i = 0; i < contacts.size(); ++i)
            {
                if (std::abs(contacts[i].penetration_depth) > deepest_penetration_depth)
                {
                    deepest_penetration_depth = std::abs(contacts[i].penetration_depth);
                    deepest_penetration_depth_index = i;
                }
            }

            const fcl::Contactd& contact = tmp_res.getContact(deepest_penetration_depth_index);
            //  By convention, Drake requires the contact normal to point out of B and
            //  into A. FCL uses the opposite convention.
            fcl::Vector3d normal = -contact.normal;

            // Signed distance is negative when penetration depth is positive.
            double signed_distance = -std::abs(contact.penetration_depth);

            if (signed_distance > 0) ThrowPretty("In collision but positive signed distance? " << signed_distance);

            // FCL returns a single contact point, but PointPair expects two, one on
            // the surface of body A (Ac) and one on the surface of body B (Bc).
            // Choose points along the line defined by the contact point and normal,
            // equi-distant to the contact point. Recall that signed_distance is
            // strictly non-positive, so signed_distance * normal points out of
            // A and into B.
            const fcl::Vector3d p_WAc{contact.pos + 0.5 * signed_distance * normal};
            const fcl::Vector3d p_WBc{contact.pos - 0.5 * signed_distance * normal};

            p.distance = signed_distance;
            p.normal1 = -contact.normal;
            p.normal2 = contact.normal;

            KDL::Vector c1 = KDL::Vector(p_WAc(0), p_WAc(1), p_WAc(2));
            KDL::Vector c2 = KDL::Vector(p_WBc(0), p_WBc(1), p_WBc(2));
            tf::vectorKDLToEigen(c1, p.contact1);
            tf::vectorKDLToEigen(c2, p.contact2);

            data->Distance = std::min(data->Distance, p.distance);
            data->proxies.push_back(p);

            return;
        }
        else
        {
            ThrowPretty("[This should not happen] In contact but did not return any contact points.");
        }
    }

    // Step 2: If not in collision, run old distance logic.
    data->request.enable_nearest_points = true;
    data->request.enable_signed_distance = true;  // Added in FCL 0.6.0 (i.e., >0.5.90)
    data->request.distance_tolerance = 1e-6;
    data->request.gjk_solver_type = fcl::GST_LIBCCD;
    data->result.clear();

    double min_dist = fcl::distance(o1, o2, data->request, data->result);

    // If -1 is returned, the returned query is a touching contact (or not implemented).
    bool touching_contact = false;
    p.distance = min_dist;
    if (min_dist != data->result.min_distance)
    {
        // This can mean this has not been implemented and results may be arbitrary.
        HIGHLIGHT_NAMED("Discrepancy", min_dist << " vs " << data->result.min_distance);
    }

    if (min_dist == -1 || std::abs(min_dist) < 1e-9)
    {
        touching_contact = true;
        p.distance = 0.0;
    }

    KDL::Vector c1, c2;

    // We need some special treatise here, cf. https://github.com/flexible-collision-library/fcl/issues/171#issuecomment-413368821
    // Case 3: Primitive vs Mesh - both returned in local frame, and additionally swapped.
    if (o1->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM && o2->getObjectType() == fcl::OBJECT_TYPE::OT_BVH)
    {
        // NOTE! The nearest points are in the wrong order now
        c1 = KDL::Vector(data->result.nearest_points[1](0), data->result.nearest_points[1](1), data->result.nearest_points[1](2));
        c2 = KDL::Vector(data->result.nearest_points[0](0), data->result.nearest_points[0](1), data->result.nearest_points[0](2));
    }
    // The default case that, in theory, should work for all cases.
    else
    {
        c1 = KDL::Vector(data->result.nearest_points[0](0), data->result.nearest_points[0](1), data->result.nearest_points[0](2));
        c2 = KDL::Vector(data->result.nearest_points[1](0), data->result.nearest_points[1](1), data->result.nearest_points[1](2));
    }

    // Check if NaN
    if (std::isnan(c1(0)) || std::isnan(c1(1)) || std::isnan(c1(2)) || std::isnan(c2(0)) || std::isnan(c2(1)) || std::isnan(c2(2)))
    {
        // LIBCCD queries require unreasonably high tolerances, i.e. we may not
        // be able to compute contacts because one of those borderline cases.
        // Hence, when we encounter a NaN for a _sphere_, we will replace it
        // with the shape centre.
        if (data->request.gjk_solver_type == fcl::GST_LIBCCD)
        {
            HIGHLIGHT_NAMED("computeDistanceLibCCD",
                            "Contact1 between " << p.e1->segment.getName() << " and " << p.e2->segment.getName() << " contains NaN"
                                                << ", where ShapeType1: " << p.e1->shape->type << " and ShapeType2: " << p.e2->shape->type << " and distance: " << p.distance << " and solver: " << data->request.gjk_solver_type);
            // To avoid downstream issues, replace contact point with shape centre
            if ((std::isnan(c1(0)) || std::isnan(c1(1)) || std::isnan(c1(2))) && p.e1->shape->type == shapes::ShapeType::SPHERE) c1 = p.e1->frame.p;
            if ((std::isnan(c2(0)) || std::isnan(c2(1)) || std::isnan(c2(2))) && p.e2->shape->type == shapes::ShapeType::SPHERE) c2 = p.e1->frame.p;
        }
        else
        {
            // TODO(#277): Any other NaN is a serious issue which we should investigate separately, so display helpful error message:
            HIGHLIGHT_NAMED("ComputeDistance",
                            "Contact1 between " << p.e1->segment.getName() << " and " << p.e2->segment.getName() << " contains NaN"
                                                << ", where ShapeType1: " << p.e1->shape->type << " and ShapeType2: " << p.e2->shape->type << " and distance: " << p.distance << " and solver: " << data->request.gjk_solver_type);
            HIGHLIGHT("c1:" << data->result.nearest_points[0](0) << "," << data->result.nearest_points[0](1) << "," << data->result.nearest_points[0](2));
            HIGHLIGHT("c2:" << data->result.nearest_points[1](0) << "," << data->result.nearest_points[1](1) << "," << data->result.nearest_points[1](2));
        }
    }

    tf::vectorKDLToEigen(c1, p.contact1);
    tf::vectorKDLToEigen(c2, p.contact2);

    // On touching contact, the normal would be ill-defined. Thus, use the shape centre of the opposite shape as a proxy contact.
    if (touching_contact)
    {
        c1 = p.e2->frame.p;
        c2 = p.e1->frame.p;
    }

    KDL::Vector n1 = c2 - c1;
    KDL::Vector n2 = c1 - c2;
    n1.Normalize();
    n2.Normalize();
    tf::vectorKDLToEigen(n1, p.normal1);
    tf::vectorKDLToEigen(n2, p.normal2);

    data->Distance = std::min(data->Distance, p.distance);
    data->proxies.push_back(p);
}

bool CollisionSceneFCLLatest::CollisionCallbackDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& dist)
{
    DistanceData* data_ = reinterpret_cast<DistanceData*>(data);

    if (!IsAllowedToCollide(o1, o2, data_->self, data_->scene)) return false;
    ComputeDistance(o1, o2, data_);
    return false;
}

bool CollisionSceneFCLLatest::IsStateValid(bool self, double safe_distance)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager(new fcl::DynamicAABBTreeCollisionManagerd());
    manager->registerObjects(fcl_objects_);
    CollisionData data(this);
    data.self = self;
    data.safe_distance = safe_distance;
    manager->collide(&data, &CollisionSceneFCLLatest::CollisionCallback);
    return !data.result.isCollision();
}

bool CollisionSceneFCLLatest::IsCollisionFree(const std::string& o1, const std::string& o2, double safe_distance)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::vector<fcl::CollisionObjectd*> shapes1;
    std::vector<fcl::CollisionObjectd*> shapes2;
    for (fcl::CollisionObjectd* o : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())].lock();
        if (e->segment.getName() == o1 || e->parent.lock()->segment.getName() == o1) shapes1.push_back(o);
        if (e->segment.getName() == o2 || e->parent.lock()->segment.getName() == o2) shapes2.push_back(o);
    }
    if (shapes1.size() == 0) ThrowPretty("Can't find object '" << o1 << "'!");
    if (shapes2.size() == 0) ThrowPretty("Can't find object '" << o2 << "'!");
    CollisionData data(this);
    data.safe_distance = safe_distance;
    for (fcl::CollisionObjectd* s1 : shapes1)
    {
        for (fcl::CollisionObjectd* s2 : shapes2)
        {
            CheckCollision(s1, s2, &data);
            if (data.result.isCollision()) return false;
        }
    }
    return true;
}

std::vector<CollisionProxy> CollisionSceneFCLLatest::GetCollisionDistance(bool self)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager(new fcl::DynamicAABBTreeCollisionManagerd());
    manager->registerObjects(fcl_objects_);
    DistanceData data(this);
    data.self = self;
    manager->distance(&data, &CollisionSceneFCLLatest::CollisionCallbackDistance);
    return data.proxies;
}

std::vector<CollisionProxy> CollisionSceneFCLLatest::GetCollisionDistance(const std::string& o1, const std::string& o2)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::vector<fcl::CollisionObjectd*> shapes1;
    std::vector<fcl::CollisionObjectd*> shapes2;
    for (fcl::CollisionObjectd* o : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())].lock();
        if (e->segment.getName() == o1 || e->parent.lock()->segment.getName() == o1) shapes1.push_back(o);
        if (e->segment.getName() == o2 || e->parent.lock()->segment.getName() == o2) shapes2.push_back(o);
    }
    if (shapes1.size() == 0) ThrowPretty("Can't find object '" << o1 << "'!");
    if (shapes2.size() == 0) ThrowPretty("Can't find object '" << o2 << "'!");
    DistanceData data(this);
    for (fcl::CollisionObjectd* s1 : shapes1)
    {
        for (fcl::CollisionObjectd* s2 : shapes2)
        {
            ComputeDistance(s1, s2, &data);
        }
    }
    return data.proxies;
}

std::vector<CollisionProxy> CollisionSceneFCLLatest::GetCollisionDistance(const std::string& o1, const bool& self)
{
    return GetCollisionDistance(o1, self, false);
}

std::vector<CollisionProxy> CollisionSceneFCLLatest::GetCollisionDistance(
    const std::string& o1, const bool& self, const bool& disable_collision_scene_update)
{
    if (!always_externally_updated_collision_scene_ && !disable_collision_scene_update) UpdateCollisionObjectTransforms();

    std::vector<fcl::CollisionObjectd*> shapes1;
    std::vector<fcl::CollisionObjectd*> shapes2;
    DistanceData data(this);
    data.self = self;

    // Iterate over all fcl_objects_ to find all collision links that belong to
    // object o1
    for (fcl::CollisionObjectd* o : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())].lock();
        if (e->segment.getName() == o1 || e->parent.lock()->segment.getName() == o1)
            shapes1.push_back(o);
    }

    // Iterate over all fcl_objects_ to find all objects o1 is allowed to collide
    // with
    for (fcl::CollisionObjectd* o : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())].lock();
        // Collision Object does not belong to o1
        if (e->segment.getName() != o1 || e->parent.lock()->segment.getName() != o1)
        {
            bool allowedToCollide = false;
            for (fcl::CollisionObjectd* o1_shape : shapes1)
                if (IsAllowedToCollide(o1_shape, o, data.self, data.scene))
                    allowedToCollide = true;

            if (allowedToCollide) shapes2.push_back(o);
        }
    }

    // There are no objects o1 is allowed to collide with, return the empty proxies vector
    if (shapes1.size() == 0 || shapes2.size() == 0) return data.proxies;

    for (fcl::CollisionObjectd* s1 : shapes1)
    {
        for (fcl::CollisionObjectd* s2 : shapes2)
        {
            ComputeDistance(s1, s2, &data);
        }
    }
    return data.proxies;
}

std::vector<CollisionProxy> CollisionSceneFCLLatest::GetCollisionDistance(const std::vector<std::string>& objects, const bool& self)
{
    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    std::vector<CollisionProxy> proxies;
    for (const auto& o1 : objects)
        AppendVector(proxies, GetCollisionDistance(o1, self, true));

    return proxies;
}

Eigen::Vector3d CollisionSceneFCLLatest::GetTranslation(const std::string& name)
{
    for (fcl::CollisionObjectd* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (element->segment.getName() == name)
        {
            return Eigen::Map<Eigen::Vector3d>(element->frame.p.data);
        }
    }
    ThrowPretty("Robot not found!");
    ;
}

std::vector<std::string> CollisionSceneFCLLatest::GetCollisionWorldLinks()
{
    std::vector<std::string> tmp;
    for (fcl::CollisionObjectd* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (!element->closest_robot_link.lock())
        {
            tmp.push_back(element->segment.getName());
        }
    }
    return tmp;
}

std::vector<std::shared_ptr<KinematicElement>> CollisionSceneFCLLatest::GetCollisionWorldLinkElements()
{
    std::vector<std::shared_ptr<KinematicElement>> tmp;
    for (fcl::CollisionObjectd* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (!element->closest_robot_link.lock())
        {
            tmp.push_back(element);
        }
    }
    return tmp;
}

std::vector<std::string> CollisionSceneFCLLatest::GetCollisionRobotLinks()
{
    std::vector<std::string> tmp;
    for (fcl::CollisionObjectd* object : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())].lock();
        if (element->closest_robot_link.lock())
        {
            tmp.push_back(element->segment.getName());
        }
    }
    return tmp;
}

ContinuousCollisionProxy CollisionSceneFCLLatest::ContinuousCollisionCheck(
    const std::string& o1, const KDL::Frame& tf1_beg, const KDL::Frame& tf1_end,
    const std::string& o2, const KDL::Frame& tf2_beg, const KDL::Frame& tf2_end)
{
    ContinuousCollisionProxy ret;

    if (!always_externally_updated_collision_scene_) UpdateCollisionObjectTransforms();

    fcl::CollisionObjectd* shape1 = nullptr;
    fcl::CollisionObjectd* shape2 = nullptr;

    for (fcl::CollisionObjectd* o : fcl_objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())].lock();
        if (e->segment.getName() == o1)
        {
            shape1 = o;
            ret.e1 = e;
        }

        if (e->segment.getName() == o2)
        {
            shape2 = o;
            ret.e2 = e;
        }
    }

    if (shape1 == nullptr) ThrowPretty("o1 not found.");
    if (shape2 == nullptr) ThrowPretty("o2 not found.");

    CollisionData data(this);
    const bool allowedToCollide = IsAllowedToCollide(shape1, shape2, data.self, data.scene);

    if (!allowedToCollide)
    {
        ret.in_collision = false;
        ret.time_of_contact = 1.0;
        return ret;
    }

    fcl::Transform3d tf1_beg_fcl = fcl_convert::KDL2fcl(tf1_beg);
    fcl::Transform3d tf1_end_fcl = fcl_convert::KDL2fcl(tf1_end);
    fcl::Transform3d tf2_beg_fcl = fcl_convert::KDL2fcl(tf2_beg);
    fcl::Transform3d tf2_end_fcl = fcl_convert::KDL2fcl(tf2_end);

    if (!tf1_beg_fcl.matrix().allFinite())
    {
        std::stringstream ss;
        ss << std::setprecision(20);
        ss << "[tf1_beg_fcl] is not finite\n"
           << tf1_beg_fcl.matrix() << "\n"
           << ToString(tf1_beg) << "\n";
        throw std::logic_error(ss.str());
    }
    if (!tf1_end_fcl.matrix().allFinite())
    {
        std::stringstream ss;
        ss << std::setprecision(20);
        ss << "[tf1_end_fcl] is not finite\n"
           << tf1_end_fcl.matrix() << "\n"
           << ToString(tf1_end) << "\n";
        throw std::logic_error(ss.str());
    }
    if (!tf2_beg_fcl.matrix().allFinite())
    {
        std::stringstream ss;
        ss << std::setprecision(20);
        ss << "[tf2_beg_fcl] is not finite\n"
           << tf2_beg_fcl.matrix() << "\n"
           << ToString(tf2_beg) << "\n";
        throw std::logic_error(ss.str());
    }
    if (!tf2_end_fcl.matrix().allFinite())
    {
        std::stringstream ss;
        ss << std::setprecision(20);
        ss << "[tf2_end_fcl] is not finite\n"
           << tf2_end_fcl.matrix() << "\n"
           << ToString(tf2_end) << "\n";
        throw std::logic_error(ss.str());
    }

    // If neither object has motion, only ran a normal collision check.
    // HIGHLIGHT_NAMED("tf1_beg_fcl", "\n" << tf1_beg_fcl.matrix());
    // HIGHLIGHT_NAMED("tf1_end_fcl", "\n" << tf1_end_fcl.matrix());
    // HIGHLIGHT_NAMED("tf2_beg_fcl", "\n" << tf2_beg_fcl.matrix());
    // HIGHLIGHT_NAMED("tf2_end_fcl", "\n" << tf2_end_fcl.matrix());
    // if (tf1_beg_fcl.isApprox(tf1_end_fcl) && tf2_beg_fcl.isApprox(tf2_end_fcl))
    // {
    //     HIGHLIGHT("Yeah, no motion here.");
    // }

    fcl::ContinuousCollisionRequestd request = fcl::ContinuousCollisionRequestd();

#ifdef CONTINUOUS_COLLISION_USE_ADVANCED_SETTINGS
    request.num_max_iterations = 100;  // default 10
    request.toc_err = 1e-5;            // default 1e-4

    // GST_LIBCCD, GST_INDEP
    // request.gjk_solver_type = fcl::GST_INDEP;

    // CCDM_TRANS, CCDM_LINEAR, CCDM_SCREW, CCDM_SPLINE
    request.ccd_motion_type = fcl::CCDMotionType::CCDM_SCREW;

    // CCDC_NAIVE, CCDC_CONSERVATIVE_ADVANCEMENT, CCDC_RAY_SHOOTING, CCDC_POLYNOMIAL_SOLVER
    // As of 2018-06-27, only CCDC_NAIVE appears to work reliably on both primitives and meshes.
    // Cf. https://github.com/flexible-collision-library/fcl/issues/120
    request.ccd_solver_type = fcl::CCDC_NAIVE;

    // If both are primitives, let's use conservative advancement
    if (shape1->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM && shape2->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM)
    {
        request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
    }
#endif

    fcl::ContinuousCollisionResultd result;
    double time_of_contact = fcl::continuousCollide(
        shape1->collisionGeometry().get(), tf1_beg_fcl, tf1_end_fcl,
        shape2->collisionGeometry().get(), tf2_beg_fcl, tf2_end_fcl,
        request, result);

#ifdef CONTINUOUS_COLLISION_DEBUG
    HIGHLIGHT_NAMED("ContinuousCollisionResult", "return=" << time_of_contact << " is_collide: " << result.is_collide << " time_of_contact: " << result.time_of_contact << " contact_tf1: " << result.contact_tf1.translation().transpose() << " contact_tf2: " << result.contact_tf2.translation().transpose());
#else
    (void)time_of_contact;
#endif

    if (result.is_collide)
    {
        if (!result.contact_tf1.matrix().allFinite())
        {
            std::stringstream ss;
            ss << "result.contact_tf1 is not finite\n"
               << result.contact_tf1.matrix() << "\n";
            throw std::logic_error(ss.str());
        }
        if (!result.contact_tf2.matrix().allFinite())
        {
            std::stringstream ss;
            ss << "result.contact_tf2 is not finite\n"
               << result.contact_tf2.matrix() << "\n";
            throw std::logic_error(ss.str());
        }
    }

    ret.in_collision = result.is_collide;
    ret.time_of_contact = result.time_of_contact;

    // If in contact, compute contact point
    if (ret.in_collision)
    {
#if 0
        // Run distance query
        fcl::DistanceRequestd distance_req;
        fcl::DistanceResultd distance_res;
        distance_req.enable_nearest_points = true;
        distance_req.enable_signed_distance = true;
        // distance_req.distance_tolerance = 1e-6;
        distance_req.gjk_solver_type = fcl::GST_LIBCCD;
        // distance_res.clear();

        double min_dist = fcl::distance(shape1, shape2, distance_req, distance_res);

        ret.penetration_depth = distance_res.min_distance;
        ret.contact_pos = distance_res.nearest_points[0];
        ret.contact_normal = (distance_res.nearest_points[1] - distance_res.nearest_points[0]).normalized();
#else
        fcl::CollisionRequestd contact_req;
        contact_req.enable_contact = true;
        contact_req.num_max_contacts = 1000;
        contact_req.gjk_tolerance = 2e-12;
        contact_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        fcl::CollisionResultd contact_res;
        size_t num_contacts = fcl::collide(shape1->collisionGeometry().get(), result.contact_tf1, shape2->collisionGeometry().get(), result.contact_tf2, contact_req, contact_res);
        if (num_contacts > 0)
        {
            std::vector<fcl::Contactd> contacts;
            contact_res.getContacts(contacts);
            ret.penetration_depth = std::numeric_limits<double>::min();
            for (const auto& contact : contacts)
            {
                if (contact.penetration_depth > ret.penetration_depth)
                {
                    if (!contact.pos.allFinite())
                    {
                        std::stringstream ss;
                        ss << std::setprecision(20);
                        ss << "Error with configuration"
                           << "\n  Shape 1: " << shape1->collisionGeometry().get()
                           << "\n  X_FS1\n"
                           << result.contact_tf1.matrix()
                           << "\n  Shape 2: " << shape2->collisionGeometry().get()
                           << "\n  X_FS2\n"
                           << result.contact_tf2.matrix()
                           << "\n  Solver: " << contact_req.gjk_solver_type;
                        throw std::logic_error(ss.str());
                    }

                    ret.penetration_depth = contact.penetration_depth;
                    ret.contact_pos = contact.pos;
                    ret.contact_normal = -contact.normal;
                }
            }
#ifdef CONTINUOUS_COLLISION_DEBUG
            HIGHLIGHT_NAMED("In collision, contacts: ", num_contacts << ", penetration=" << ret.penetration_depth << ", pos: " << ret.contact_pos.transpose());
#endif
        }
        else
        {
            ret.penetration_depth = 0.0;
            ret.in_collision = false;
        }
#endif
    }

    tf::transformEigenToKDL(static_cast<Eigen::Isometry3d>(result.contact_tf1), ret.contact_tf1);
    tf::transformEigenToKDL(static_cast<Eigen::Isometry3d>(result.contact_tf2), ret.contact_tf2);

#ifdef CONTINUOUS_COLLISION_DEBUG
    if (!ret.contact_pos.allFinite())
    {
        ThrowPretty("Contact position is not finite!");
    }
#endif

    return ret;
}
}  // namespace exotica
