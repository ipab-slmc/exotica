/*
 *      Author: Wolfgang Merkt
 *
 * Copyright (c) 2017, Wolfgang Merkt
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

#include <collision_scene_bullet/CollisionSceneBullet.h>
#include <exotica/Factory.h>

REGISTER_COLLISION_SCENE_TYPE("CollisionSceneBullet", exotica::CollisionSceneBullet)

namespace bullet_convert
{
btTransform KDL2Bullet(const KDL::Frame& frame)
{
    btTransform ret;
    ret.setIdentity();
    ret.setOrigin(btVector3(frame.p[0], frame.p[1], frame.p[2]));
    double qx, qy, qz, qw;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    ret.setRotation(btQuaternion(qx, qy, qz, qw));
    return ret;
}
}

namespace exotica
{
CollisionSceneBullet::CollisionSceneBullet()
    : bt_collision_configuration_(),
      bt_collision_broadphase_()
{
    HIGHLIGHT("Bullet version: " << btGetVersion());

    bt_collision_configuration_.setConvexConvexMultipointIterations(0, 0);
    bt_collision_configuration_.setPlaneConvexMultipointIterations(0, 0);
    bt_collision_dispatcher_.reset(new btCollisionDispatcher(&bt_collision_configuration_));
    // bt_collision_world_.reset(new btCollisionWorld(bt_collision_dispatcher_.get(), &bt_collision_broadphase_, &bt_collision_configuration_));

    // TODO(wxm): overlap filter
    // bt_collision_world->getPairCache()->setOverlapFilterCallback(&filter_callback_);
}

CollisionSceneBullet::~CollisionSceneBullet()
{
}

void CollisionSceneBullet::updateCollisionObjects(const std::map<std::string, std::shared_ptr<KinematicElement>>& objects)
{
    // TODO: add cache/something more intelligent here

    // Step 1: Remove all existing collision objects in the world
    bt_collision_world_.reset(new btCollisionWorld(bt_collision_dispatcher_.get(), &bt_collision_broadphase_, &bt_collision_configuration_));
    // const auto& current_collision_objects = bt_collision_world_->getCollisionObjectArray();
    // for (size_t i = 0; i < current_collision_objects.size(); ++i)
    // {
    //     std::cout << "Deleting #" << i << std::endl;
    //     bt_collision_world_->removeCollisionObject(current_collision_objects.at(i));
    // }

    // Step 2: Create and add all 'new'/current collision objects
    kinematic_elements_ = MapToVec(objects);
    std::cout << "Expecting " << objects.size() << " collision objects" << std::endl;
    long i = 0;
    for (const auto& object : objects)
    {
        std::cout << "Updating " << object.first << std::endl;
        std::unique_ptr<btCollisionShape> new_shape = constructBulletCollisionObject(object.second);

        if (!new_shape) throw_pretty("Bullet Collision Shape broken :(");
        // std::cout << "POS: " << Eigen::Vector3d(object.second->Frame.p[0], object.second->Frame.p[1], object.second->Frame.p[2]).transpose() << std::endl;
        // btTransform btPos = bullet_convert::KDL2Bullet(object.second->Frame);
        // std::cout << "btPOS: " << Eigen::Vector3d(btPos.getOrigin().getX(), btPos.getOrigin().getY(), btPos.getOrigin().getZ()).transpose() << std::endl;

        // Create actual collision object
        std::unique_ptr<btCollisionObject> bt_obj(new btCollisionObject());
        bt_obj->setCollisionShape(new_shape.get());
        // bt_obj->setWorldTransform(bullet_convert::KDL2Bullet(object.second->Frame));
        // bt_obj->setWorldTransform(btPos);

        // bt_obj->setUserPointer(object.second.get());
        bt_obj->setUserPointer(reinterpret_cast<void*>(i));
        //
        // OKAY TEST THIS MYSELF WTF
        // btVector3  minAabb;
        // btVector3  maxAabb;
        // bt_obj->getCollisionShape()->getAabb(bt_obj->getWorldTransform(),minAabb,maxAabb);
        // std::cout << "minAabb: " << minAabb.x() << "," << minAabb.y() << "," << minAabb.z() << std::endl;
        // std::cout << "maxAabb: " << maxAabb.x() << "," << maxAabb.y() << "," << maxAabb.z() << std::endl;

        // std::cout << "r=" << dynamic_cast<btSphereShape>(bt_obj->getCollisionShape())->getRadius() << std::endl;
        // std::cout << "margin=" << bt_obj->getCollisionShape()->getMargin() << std::endl;

        if (!bt_obj) throw_pretty("Bullet Collision Object broken :(");

        std::cout << "Let's add this beast" << std::endl;

        // TODO: these collision filters are awful for static objects!!
        bt_collision_world_->addCollisionObject(bt_obj.get());  //, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
        std::cout << "I believe I can fly" << std::endl;
        // bullet_collision_objects_[object.first] = move(bt_obj);
        i++;

        // // Take ownership of collision shapes
        // bullet_collision_shapes_.push_back(move(new_shape));
    }
}

void CollisionSceneBullet::updateCollisionObjectTransforms()
{
    std::cout << "update collision transforms from kineatmics" << std::endl;
    // for (auto& collision_object : bullet_collision_objects_)
    // {
    //     std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(collision_object->getUserPointer())];
    //     std::cout << "Updating transform for " << element->Segment.getName() << std::endl;
    //     btTransform tmp_transform;
    //     tmp_transform.setIdentity();
    //     collision_object->setWorldTransform(tmp_transform);
    //     // collision_object->setWorldTransform(bullet_convert::KDL2Bullet(element->Frame));
    //     // bt_collision_world_->updateSingleAabb(collision_object.get());
    // }
    // std::cout << "Updating AABBs" << std::endl;
    // bt_collision_world_->updateAabbs();
}

std::unique_ptr<btCollisionShape> CollisionSceneBullet::constructBulletCollisionObject(std::shared_ptr<KinematicElement> element)
{
    // Maybe use cache here?
    shapes::ShapeConstPtr shape = element->Shape;

    // Apply scaling and padding
    if (element->isRobotLink || element->ClosestRobotLink)
    {
        if (robotLinkScale_ != 1.0 || robotLinkPadding_ > 0.0)
        {
            shapes::ShapePtr scaled_shape(shape->clone());
            scaled_shape->scaleAndPadd(robotLinkScale_, robotLinkPadding_);
            shape = scaled_shape;
        }
    }
    else
    {
        if (worldLinkScale_ != 1.0 || worldLinkPadding_ > 0.0)
        {
            shapes::ShapePtr scaled_shape(shape->clone());
            scaled_shape->scaleAndPadd(worldLinkScale_, worldLinkPadding_);
            shape = scaled_shape;
        }
    }

    std::unique_ptr<btCollisionShape> bt_shape;
    switch (shape->type)
    {
        case shapes::PLANE:
        {
            throw_pretty("Plane is currently unsupported");
            // const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
            // geometry.reset(new fcl::Planed(p->a, p->b, p->c, p->d));
        }
        break;
        case shapes::SPHERE:
        {
            const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
            std::cout << "Creating sphere with radius: " << s->radius << std::endl;
            bt_shape.reset(new btSphereShape((double)s->radius));
        }
        break;
        case shapes::BOX:
        {
            const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
            const double* size = s->size;

            // Copied from https://github.com/RobotLocomotion/drake/blob/master/drake/multibody/collision/bullet_model.cc#L112
            bt_shape.reset(new btConvexHullShape());
            btBoxShape bt_box(btVector3(size[0] / 2., size[1] / 2., size[2] / 2.));
            /* Strange things happen to the collision-normals when we use the
               * convex interface to the btBoxShape. Instead, we'll explicitly create
               * a btConvexHullShape.
               */
            bt_shape->setMargin(1e-9);
            for (int i = 0; i < 8; ++i)
            {
                btVector3 vtx;
                bt_box.getVertex(i, vtx);
                dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(vtx);
            }
        }
        break;
        case shapes::CYLINDER:
        {
            const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
            bt_shape.reset(new btCylinderShapeZ(btVector3(s->radius, s->radius, s->length / 2)));
        }
        break;
        case shapes::CONE:
        {
            throw_pretty("Cone is currently unsupported");
            // const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
            // geometry.reset(new fcl::Coned(s->radius, s->length));
        }
        break;
        case shapes::MESH:
        {
            throw_pretty("Mesh is currently unsupported");
            // fcl::BVHModel<fcl::OBBRSSd>* g = new fcl::BVHModel<fcl::OBBRSSd>();
            // const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
            // if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
            // {
            //     std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
            //     for (unsigned int i = 0; i < mesh->triangle_count; ++i)
            //         tri_indices[i] =
            //             fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

            //     std::vector<fcl::Vector3d> points(mesh->vertex_count);
            //     for (unsigned int i = 0; i < mesh->vertex_count; ++i)
            //         points[i] = fcl::Vector3d(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

            //     g->beginModel();
            //     g->addSubModel(points, tri_indices);
            //     g->endModel();
            // }
            // geometry.reset(g);
        }
        break;
        case shapes::OCTREE:
        {
            throw_pretty("OcTree is currently unsupported");
            // const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
            // geometry.reset(new fcl::OcTreed(to_std_ptr(g->octree)));
        }
        break;
        default:
            throw_pretty("This shape type (" << ((int)shape->type) << ") is not supported using Bullet yet");
    }

    return bt_shape;
}

// bool CollisionSceneBullet::isAllowedToCollide(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, bool self, CollisionSceneBullet* scene)
// {
//     std::shared_ptr<KinematicElement> e1 = scene->kinematic_elements_[reinterpret_cast<long>(o1->getUserData())];
//     std::shared_ptr<KinematicElement> e2 = scene->kinematic_elements_[reinterpret_cast<long>(o2->getUserData())];

//     bool isRobot1 = e1->isRobotLink || e1->ClosestRobotLink;
//     bool isRobot2 = e2->isRobotLink || e2->ClosestRobotLink;
//     // Don't check collisions between world objects
//     if (!isRobot1 && !isRobot2) return false;
//     // Skip self collisions if requested
//     if (isRobot1 && isRobot2 && !self) return false;
//     // Skip collisions between shapes within the same objects
//     if (e1->Parent == e2->Parent) return false;
//     // Skip collisions between bodies attached to the same object
//     if (e1->ClosestRobotLink && e2->ClosestRobotLink && e1->ClosestRobotLink == e2->ClosestRobotLink) return false;

//     if (isRobot1 && isRobot2)
//     {
//         const std::string& name1 = e1->ClosestRobotLink ? e1->ClosestRobotLink->Segment.getName() : e1->Parent->Segment.getName();
//         const std::string& name2 = e2->ClosestRobotLink ? e2->ClosestRobotLink->Segment.getName() : e2->Parent->Segment.getName();
//         return scene->acm_.getAllowedCollision(name1, name2);
//     }
//     return true;
// }

// void CollisionSceneBullet::checkCollision(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, CollisionData* data)
// {
//     data->Request.num_max_contacts = 1000;
//     data->Request.gjk_solver_type = fcl::GST_INDEP;  // CCD returns wrong points
//     data->Result.clear();
//     fcl::collide(o1, o2, data->Request, data->Result);
//     if (data->SafeDistance > 0.0 && o1->getAABB().distance(o2->getAABB()) < data->SafeDistance)
//     {
//         fcl::DistanceRequestd req;
//         fcl::DistanceResultd res;
//         req.enable_nearest_points = false;
//         fcl::distance(o1, o2, req, res);
//         // Add fake contact when distance is smaller than the safety distance.
//         if (res.min_distance < data->SafeDistance) data->Result.addContact(fcl::Contactd());
//     }
// }

// bool CollisionSceneBullet::collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
// {
//     CollisionData* data_ = reinterpret_cast<CollisionData*>(data);

//     if (!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;

//     checkCollision(o1, o2, data_);
//     return data_->Result.isCollision();
// }

// void CollisionSceneBullet::computeDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, DistanceData* data)
// {
//     data->Request.enable_nearest_points = true;
//     data->Request.enable_signed_distance = true;  // Added in FCL 0.6.0
//     // GST_LIBCCD produces incorrect contacts. Probably due to incompatible version of libccd.
//     // However, FCL code comments suggest INDEP producing incorrect contact points, cf.
//     // https://github.com/flexible-collision-library/fcl/blob/master/test/test_fcl_signed_distance.cpp#L85-L86

//     // INDEP better for primitives, CCD better for when there's a mesh
//     HIGHLIGHT_NAMED("Types", "o1: " << o1->getObjectType() << " and o2: " << o2->getObjectType());
//     HIGHLIGHT("fcl::OBJECT_TYPE::OT_GEOM: " << fcl::OBJECT_TYPE::OT_GEOM);
//     if (false)  //(o1->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM && o2->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM)
//     {
//         HIGHLIGHT("using indep");
//         data->Request.gjk_solver_type = fcl::GST_INDEP;
//     }
//     else
//     {
//         HIGHLIGHT("using libccd");
//         data->Request.gjk_solver_type = fcl::GST_LIBCCD;
//     }

//     data->Result.clear();

//     // Nearest point calculation is broken when o1 is a primitive and o2 a mesh,
//     // works however for o1 being a mesh and o2 being a primitive. Due to this,
//     // we will flip the order of o1 and o2 in the request and then later on swap
//     // the contact points and normals.
//     // Cf. Issue #184:
//     // https://github.com/ipab-slmc/exotica/issues/184#issuecomment-341916457
//     bool flipO1AndO2 = false;
//     if (o1->getObjectType() == fcl::OBJECT_TYPE::OT_GEOM && o2->getObjectType() == fcl::OBJECT_TYPE::OT_BVH)
//     {
//         // HIGHLIGHT_NAMED("CollisionSceneBullet", "Flipping o1 and o2");
//         flipO1AndO2 = true;
//         fcl::distance(o2, o1, data->Request, data->Result);
//     }
//     else
//     {
//         fcl::distance(o1, o2, data->Request, data->Result);
//     }

//     CollisionProxy p;
//     p.e1 = data->Scene->kinematic_elements_[reinterpret_cast<long>(o1->getUserData())];
//     p.e2 = data->Scene->kinematic_elements_[reinterpret_cast<long>(o2->getUserData())];

//     p.distance = data->Result.min_distance;

//     // FCL uses world coordinates for meshes while local coordinates are used
//     // for primitive shapes - thus, we need to work around this.
//     // Cf. https://github.com/flexible-collision-library/fcl/issues/171
//     //
//     // Additionally, when in penetration (distance < 0), for meshes, contact
//     // points are reasonably accurate. Contact points for primitives are not
//     // reliable (FCL bug), use shape centres instead.
//     KDL::Vector c1, c2;

//     // Case 1: Mesh vs Mesh - already in world frame
//     if (p.e1->Shape->type == shapes::ShapeType::MESH && p.e2->Shape->type == shapes::ShapeType::MESH)
//     {
//         c1 = KDL::Vector(data->Result.nearest_points[0](0), data->Result.nearest_points[0](1), data->Result.nearest_points[0](2));
//         c2 = KDL::Vector(data->Result.nearest_points[1](0), data->Result.nearest_points[1](1), data->Result.nearest_points[1](2));
//     }
//     // Case 2: Primitive vs Primitive - convert from both local frames to world frame
//     else if (p.e1->Shape->type != shapes::ShapeType::MESH && p.e2->Shape->type != shapes::ShapeType::MESH)
//     {
//         // Use shape centres as nearest point when in penetration - otherwise use the nearest point.
//         // INDEP has a further caveat when in penetration: it will return the
//         // exact touch location as the contact point - not the point of maximum
//         // penetration. This contact point will be in world frame, while the
//         // closest point is in local frame.
//         if (p.distance > 0)
//         {
//             HIGHLIGHT("both are primitives so let's use whatever comes back");
//             c1 = p.e1->Frame * KDL::Vector(data->Result.nearest_points[0](0), data->Result.nearest_points[0](1), data->Result.nearest_points[0](2));
//             c2 = p.e2->Frame * KDL::Vector(data->Result.nearest_points[1](0), data->Result.nearest_points[1](1), data->Result.nearest_points[1](2));
//         }
//         else
//         {
//             // The contact point is accurate but it is not the point of deepest penetration
//             // WITH INDEP:
//             // c1 = KDL::Vector(data->Result.nearest_points[0](0), data->Result.nearest_points[0](1), data->Result.nearest_points[0](2));
//             // c2 = KDL::Vector(data->Result.nearest_points[1](0), data->Result.nearest_points[1](1), data->Result.nearest_points[1](2));

//             // WITH LIBCCD:
//             c1 = p.e1->Frame * KDL::Vector(data->Result.nearest_points[0](0), data->Result.nearest_points[0](1), data->Result.nearest_points[0](2));
//             c2 = p.e2->Frame * KDL::Vector(data->Result.nearest_points[1](0), data->Result.nearest_points[1](1), data->Result.nearest_points[1](2));

//             // SHAPE CENTRES:
//             // c1 = p.e1->Frame.p;
//             // c2 = p.e2->Frame.p;
//         }
//     }
//     // Case 3: Primitive vs Mesh - primitive returned in flipped local frame (tbc), mesh returned in global frame
//     else if (p.e1->Shape->type != shapes::ShapeType::MESH && p.e2->Shape->type == shapes::ShapeType::MESH)
//     {
//         // Flipping contacts is a workaround for issue #184
//         // Cf. https://github.com/ipab-slmc/exotica/issues/184#issuecomment-341916457
//         if (!flipO1AndO2) throw_pretty("We got the broken case but aren't flipping, why?");

//         // Note: e1 and e2 are swapped, i.e. c1 on e2, c2 on e1
//         // e2 is a mesh, always fine
//         c1 = p.e2->Frame * KDL::Vector(data->Result.nearest_points[0](0), data->Result.nearest_points[0](1), data->Result.nearest_points[0](2));

//         // e1 is a primitive, i.e. use shape centres as nearest point when in penetration
//         if (p.distance > 0)
//         {
//             c2 = p.e1->Frame * KDL::Vector(data->Result.nearest_points[1](0), data->Result.nearest_points[1](1), data->Result.nearest_points[1](2));
//         }
//         else
//         {
//             c2 = p.e1->Frame.p;
//         }
//     }
//     // Case 4: Mesh vs Primitive - both are returned in the local frame (works with both LIBCCD and INDEP)
//     else if (p.e1->Shape->type == shapes::ShapeType::MESH && p.e2->Shape->type != shapes::ShapeType::MESH)
//     {
//         // e1 is mesh, i.e. nearest points are fine
//         c1 = p.e1->Frame * KDL::Vector(data->Result.nearest_points[0](0), data->Result.nearest_points[0](1), data->Result.nearest_points[0](2));

//         // e2 is a primitive, i.e. use shape centres as nearest point when in penetration
//         if (p.distance > 0)
//         {
//             c2 = p.e2->Frame * KDL::Vector(data->Result.nearest_points[1](0), data->Result.nearest_points[1](1), data->Result.nearest_points[1](2));
//         }
//         else
//         {
//             c2 = p.e2->Frame.p;
//         }
//     }
//     // Unknown case - what's up?
//     else
//     {
//         throw_pretty("e1: " << p.e1->Shape->type << " vs e2: " << p.e2->Shape->type);
//     }

//     if (flipO1AndO2)
//     {
//         // Flipping contacts is a workaround for issue #184
//         // Cf. https://github.com/ipab-slmc/exotica/issues/184#issuecomment-341916457
//         KDL::Vector tmp_c1 = KDL::Vector(c1);
//         KDL::Vector tmp_c2 = KDL::Vector(c2);
//         c1 = tmp_c2;
//         c2 = tmp_c1;
//     }

//     KDL::Vector n1 = c2 - c1;
//     KDL::Vector n2 = c1 - c2;
//     n1.Normalize();
//     n2.Normalize();
//     tf::vectorKDLToEigen(c1, p.contact1);
//     tf::vectorKDLToEigen(c2, p.contact2);
//     tf::vectorKDLToEigen(n1, p.normal1);
//     tf::vectorKDLToEigen(n2, p.normal2);

//     data->Distance = std::min(data->Distance, p.distance);
//     data->Proxies.push_back(p);
// }

// bool CollisionSceneBullet::collisionCallbackDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& dist)
// {
//     DistanceData* data_ = reinterpret_cast<DistanceData*>(data);

//     if (!isAllowedToCollide(o1, o2, data_->Self, data_->Scene)) return false;
//     computeDistance(o1, o2, data_);
//     return false;
// }

bool CollisionSceneBullet::isStateValid(bool self, double safe_distance)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();
    throw_pretty("NOPE");

    // std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager(new fcl::DynamicAABBTreeCollisionManagerd());
    // manager->registerObjects(bullet_objects_);
    CollisionData data(this);
    data.Self = self;
    data.SafeDistance = safe_distance;
    // manager->collide(&data, &CollisionSceneBullet::collisionCallback);
    return false;  //!data.Result.isCollision();
}

bool CollisionSceneBullet::isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();

    // std::vector<fcl::CollisionObjectd*> shapes1;
    // std::vector<fcl::CollisionObjectd*> shapes2;
    // for (fcl::CollisionObjectd* o : bullet_objects_)
    // {
    //     std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())];
    //     if (e->Segment.getName() == o1 || e->Parent->Segment.getName() == o1) shapes1.push_back(o);
    //     if (e->Segment.getName() == o2 || e->Parent->Segment.getName() == o2) shapes2.push_back(o);
    // }
    // if (shapes1.size() == 0) throw_pretty("Can't find object '" << o1 << "'!");
    // if (shapes2.size() == 0) throw_pretty("Can't find object '" << o2 << "'!");
    // CollisionData data(this);
    // data.SafeDistance = safe_distance;
    // for (fcl::CollisionObjectd* s1 : shapes1)
    // {
    //     for (fcl::CollisionObjectd* s2 : shapes2)
    //     {
    //         checkCollision(s1, s2, &data);
    //         if (data.Result.isCollision()) return false;
    //     }
    // }
    return true;
}

std::vector<CollisionProxy> CollisionSceneBullet::getCollisionDistance(bool self)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();

    //     std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager(new fcl::DynamicAABBTreeCollisionManagerd());
    //     manager->registerObjects(bullet_objects_);
    DistanceData data(this);
    data.Self = self;
    //     manager->distance(&data, &CollisionSceneBullet::collisionCallbackDistance);
    return data.Proxies;
}

std::vector<CollisionProxy> CollisionSceneBullet::getCollisionDistance(const std::string& o1, const std::string& o2)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();

    //     std::vector<fcl::CollisionObjectd*> shapes1;
    //     std::vector<fcl::CollisionObjectd*> shapes2;
    //     for (fcl::CollisionObjectd* o : bullet_objects_)
    //     {
    //         std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())];
    //         if (e->Segment.getName() == o1 || e->Parent->Segment.getName() == o1) shapes1.push_back(o);
    //         if (e->Segment.getName() == o2 || e->Parent->Segment.getName() == o2) shapes2.push_back(o);
    //     }
    //     if (shapes1.size() == 0) throw_pretty("Can't find object '" << o1 << "'!");
    //     if (shapes2.size() == 0) throw_pretty("Can't find object '" << o2 << "'!");
    DistanceData data(this);
    //     for (fcl::CollisionObjectd* s1 : shapes1)
    //     {
    //         for (fcl::CollisionObjectd* s2 : shapes2)
    //         {
    //             computeDistance(s1, s2, &data);
    //         }
    //     }
    return data.Proxies;
}

std::vector<CollisionProxy> CollisionSceneBullet::getCollisionDistance(const std::string& o1, const bool& self)
{
    return getCollisionDistance(o1, self, false);
}

std::vector<CollisionProxy> CollisionSceneBullet::getCollisionDistance(
    const std::string& o1, const bool& self, const bool& disableCollisionSceneUpdate)
{
    if (!alwaysExternallyUpdatedCollisionScene_ && !disableCollisionSceneUpdate) updateCollisionObjectTransforms();

    //     std::vector<fcl::CollisionObjectd*> shapes1;
    //     std::vector<fcl::CollisionObjectd*> shapes2;
    DistanceData data(this);
    data.Self = self;

    //     // Iterate over all bullet_objects_ to find all collision links that belong to
    //     // object o1
    //     for (fcl::CollisionObjectd* o : bullet_objects_)
    //     {
    //         std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())];
    //         if (e->Segment.getName() == o1 || e->Parent->Segment.getName() == o1)
    //             shapes1.push_back(o);
    //     }

    //     // Iterate over all bullet_objects_ to find all objects o1 is allowed to collide
    //     // with
    //     for (fcl::CollisionObjectd* o : bullet_objects_)
    //     {
    //         std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o->getUserData())];
    //         // Collision Object does not belong to o1
    //         if (e->Segment.getName() != o1 || e->Parent->Segment.getName() != o1)
    //         {
    //             bool allowedToCollide = false;
    //             for (fcl::CollisionObjectd* o1_shape : shapes1)
    //                 if (isAllowedToCollide(o1_shape, o, data.Self, data.Scene))
    //                     allowedToCollide = true;

    //             if (allowedToCollide) shapes2.push_back(o);
    //         }
    //     }

    //     if (shapes1.size() == 0) throw_pretty("Can't find object '" << o1 << "'!");

    //     // There are no objects o1 is allowed to collide with, return the empty proxies vector
    //     if (shapes2.size() == 0) return data.Proxies;

    //     for (fcl::CollisionObjectd* s1 : shapes1)
    //     {
    //         for (fcl::CollisionObjectd* s2 : shapes2)
    //         {
    //             computeDistance(s1, s2, &data);
    //         }
    //     }
    return data.Proxies;
}

std::vector<CollisionProxy> CollisionSceneBullet::getCollisionDistance(const std::vector<std::string>& objects, const bool& self)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();

    std::vector<CollisionProxy> proxies;
    for (const auto& o1 : objects)
        appendVector(proxies, getCollisionDistance(o1, self, true));

    return proxies;
}

Eigen::Vector3d CollisionSceneBullet::getTranslation(const std::string& name)
{
    // for (fcl::CollisionObjectd* object : bullet_objects_)
    // {
    //     std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())];
    //     if (element->Segment.getName() == name)
    //     {
    //         return Eigen::Map<Eigen::Vector3d>(element->Frame.p.data);
    //     }
    // }
    throw_pretty("Robot not found!");
}

std::vector<std::string> CollisionSceneBullet::getCollisionWorldLinks()
{
    std::vector<std::string> tmp;
    // for (fcl::CollisionObjectd* object : bullet_objects_)
    // {
    //     std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())];
    //     if (!element->ClosestRobotLink)
    //     {
    //         tmp.push_back(element->Segment.getName());
    //     }
    // }
    return tmp;
}

/**
   * @brief      Gets the collision robot links.
   *
   * @return     The collision robot links.
   */
std::vector<std::string> CollisionSceneBullet::getCollisionRobotLinks()
{
    std::vector<std::string> tmp;
    // for (fcl::CollisionObjectd* object : bullet_objects_)
    // {
    //     std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object->getUserData())];
    //     if (element->ClosestRobotLink)
    //     {
    //         tmp.push_back(element->Segment.getName());
    //     }
    // }
    return tmp;
}
}
