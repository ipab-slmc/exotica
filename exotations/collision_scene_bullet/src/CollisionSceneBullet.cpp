/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2018, University Of Edinburgh
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

namespace exotica
{

btTransform KDL2bt(const KDL::Frame& frame)
{
    return btTransform(btMatrix3x3(frame.M.data[0], frame.M.data[1], frame.M.data[2], frame.M.data[3], frame.M.data[4], frame.M.data[5], frame.M.data[6], frame.M.data[7], frame.M.data[8]), btVector3(frame.p[0], frame.p[1], frame.p[2]));
}


void MyNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
    CollisionSceneBullet* scene = static_cast<CollisionSceneBullet::btExoticaCollisionConfiguration*>(dispatcher.getCollisionConfiguration())->scene_;
    btCollisionObject* o1 = reinterpret_cast<btCollisionObject*>(collisionPair.m_pProxy0->m_clientObject);
    btCollisionObject* o2 = reinterpret_cast<btCollisionObject*>(collisionPair.m_pProxy1->m_clientObject);

    // Do your collision logic here
    // Only dispatch the Bullet collision information if you want the physics to continue
    if(scene->isAllowedToCollide(o1, o2, static_cast<CollisionSceneBullet::btExoticaCollisionConfiguration*>(dispatcher.getCollisionConfiguration())->self, scene))
        dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}

CollisionSceneBullet::CollisionSceneBullet()
{
    bt_collision_configuration = std::unique_ptr<btExoticaCollisionConfiguration>(new btExoticaCollisionConfiguration(this));
    bt_dispatcher = std::unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(bt_collision_configuration.get()));
    bt_dispatcher->setNearCallback(&MyNearCallback);
    bt_broadphase = std::unique_ptr<btDbvtBroadphase>(new btDbvtBroadphase());
    bt_collision_world = std::unique_ptr<btCollisionWorld>(new btCollisionWorld(bt_dispatcher.get(), bt_broadphase.get(), bt_collision_configuration.get()));

    HIGHLIGHT("Using BUllet collision checker");
}

CollisionSceneBullet::~CollisionSceneBullet()
{
}

void CollisionSceneBullet::updateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects)
{
    kinematic_elements_ = MapToVec(objects);
    bt_collision_world->getCollisionObjectArray().clear();
    objects_.clear();
    shapes_.clear();
    meshes_.clear();
    long i = 0;
    for (const auto& object : objects)
    {
        std::shared_ptr<btCollisionObject> new_object;
        new_object = constructBulletCollisionObject(i++, object.second.lock());
        bt_collision_world->addCollisionObject(new_object.get());
        objects_[object.first] = new_object;
    }
}

void CollisionSceneBullet::updateCollisionObjectTransforms()
{
    for(auto& object : objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object.second->getUserPointer())].lock();
        if (!element)
        {
            throw_pretty("Expired pointer, this should not happen - make sure to call updateCollisionObjects() after updateSceneFrames()");
        }

        object.second->setWorldTransform(KDL2bt(element->Frame));
    }
}

std::shared_ptr<btCollisionObject> CollisionSceneBullet::constructBulletCollisionObject(long i, std::shared_ptr<KinematicElement> element)
{
    shapes::ShapeConstPtr shape = element->Shape;

    // Apply scaling and padding
    if (element->isRobotLink || element->ClosestRobotLink.lock())
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

    std::shared_ptr<btCollisionObject> ret(new btCollisionObject());
    ret->setUserPointer(reinterpret_cast<void*>(i));

    std::shared_ptr<btCollisionShape> bshape;

    switch (shape->type)
    {
        case shapes::PLANE:
        {
            const shapes::Plane* p = static_cast<const shapes::Plane*>(shape.get());
            throw_pretty("Plane shape is not implemented!");
        }
        break;
        case shapes::SPHERE:
        {
            const shapes::Sphere* s = static_cast<const shapes::Sphere*>(shape.get());
            bshape.reset(new btSphereShape(s->radius));
        }
        break;
        case shapes::BOX:
        {
            const shapes::Box* s = static_cast<const shapes::Box*>(shape.get());
            bshape.reset(new btBoxShape(btVector3(s->size[0]*0.5, s->size[1]*0.5, s->size[2]*0.5)));
        }
        break;
        case shapes::CYLINDER:
        {
            const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(shape.get());
            bshape.reset(new btCylinderShape(btVector3(s->radius, s->length, 0.0)));
        }
        break;
        case shapes::CONE:
        {
            const shapes::Cone* s = static_cast<const shapes::Cone*>(shape.get());
            bshape.reset(new btConeShape(s->radius, s->length));
        }
        break;
        case shapes::MESH:
        {
            const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());

            std::shared_ptr<btTriangleMesh> bmesh(new btTriangleMesh(false));
            if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
            {
                for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                    bmesh->findOrAddVertex(btVector3(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]), false);

                for (unsigned int i = 0; i < mesh->triangle_count*3; ++i)
                {
                    bmesh->getIndexedMeshArray()[0].m_numVertices++;
                    bmesh->addIndex(mesh->triangles[i]);
                }
            }
            bshape.reset(new btBvhTriangleMeshShape(bmesh.get(), false));
            meshes_.push_back(bmesh);
        }
        break;
        case shapes::OCTREE:
        {
            const shapes::OcTree* g = static_cast<const shapes::OcTree*>(shape.get());
            throw_pretty("OCTREE collision shape not implemented!");
        }
        break;
        default:
            throw_pretty("This shape type (" << ((int)shape->type) << ") is not supported using FCL yet");
    }

    ret->setCollisionShape(bshape.get());
    shapes_.push_back(bshape);

    return ret;
}

bool CollisionSceneBullet::isAllowedToCollide(const btCollisionObject* o1, const btCollisionObject* o2, bool self, CollisionSceneBullet* scene)
{
    std::shared_ptr<KinematicElement> e1 = scene->kinematic_elements_[reinterpret_cast<long>(o1->getUserPointer())].lock();
    std::shared_ptr<KinematicElement> e2 = scene->kinematic_elements_[reinterpret_cast<long>(o2->getUserPointer())].lock();

    bool isRobot1 = e1->isRobotLink || e1->ClosestRobotLink.lock();
    bool isRobot2 = e2->isRobotLink || e2->ClosestRobotLink.lock();
    // Don't check collisions between world objects
    if (!isRobot1 && !isRobot2) return false;
    // Skip self collisions if requested
    if (isRobot1 && isRobot2 && !self) return false;
    // Skip collisions between shapes within the same objects
    if (e1->Parent.lock() == e2->Parent.lock()) return false;
    // Skip collisions between bodies attached to the same object
    if (e1->ClosestRobotLink.lock() && e2->ClosestRobotLink.lock() && e1->ClosestRobotLink.lock() == e2->ClosestRobotLink.lock()) return false;

    if (isRobot1 && isRobot2)
    {
        const std::string& name1 = e1->ClosestRobotLink.lock() ? e1->ClosestRobotLink.lock()->Segment.getName() : e1->Parent.lock()->Segment.getName();
        const std::string& name2 = e2->ClosestRobotLink.lock() ? e2->ClosestRobotLink.lock()->Segment.getName() : e2->Parent.lock()->Segment.getName();
        return scene->acm_.getAllowedCollision(name1, name2);
    }
    return true;
}

bool CollisionSceneBullet::isStateValid(bool self, double safe_distance)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();
    static_cast<btExoticaCollisionConfiguration*>(bt_collision_configuration.get())->self = self;
    bt_collision_world->performDiscreteCollisionDetection();
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        if(contactManifold->getNumContacts()>0) return false;
    }
    return true;
}

bool CollisionSceneBullet::isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance)
{
    if (!alwaysExternallyUpdatedCollisionScene_) updateCollisionObjectTransforms();

    std::vector<std::shared_ptr<btCollisionObject>> shapes1;
    std::vector<std::shared_ptr<btCollisionObject>> shapes2;
    for (auto o : objects_)
    {
        std::shared_ptr<KinematicElement> e = kinematic_elements_[reinterpret_cast<long>(o.second->getUserPointer())].lock();
        if (e->Segment.getName() == o1 || e->Parent.lock()->Segment.getName() == o1) shapes1.push_back(o.second);
        if (e->Segment.getName() == o2 || e->Parent.lock()->Segment.getName() == o2) shapes2.push_back(o.second);
    }
    if (shapes1.size() == 0) throw_pretty("Can't find object '" << o1 << "'!");
    if (shapes2.size() == 0) throw_pretty("Can't find object '" << o2 << "'!");

    //CollisionData data(this);
    //data.SafeDistance = safe_distance;

    CollisionData ret;

    for (auto s1 : shapes1)
    {
        for (auto s2 : shapes2)
        {
            bt_collision_world->contactPairTest(s1.get(), s2.get(), ret);
            if (ret.isColliding) return false;
        }
    }
    return true;
}

Eigen::Vector3d CollisionSceneBullet::getTranslation(const std::string& name)
{
    for (auto object : objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object.second->getUserPointer())].lock();
        if (element->Segment.getName() == name)
        {
            return Eigen::Map<Eigen::Vector3d>(element->Frame.p.data);
        }
    }
    throw_pretty("Robot not found!");
}

std::vector<std::string> CollisionSceneBullet::getCollisionWorldLinks()
{
    std::vector<std::string> tmp;
    for (auto object : objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object.second->getUserPointer())].lock();
        if (!element->ClosestRobotLink.lock())
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
std::vector<std::string> CollisionSceneBullet::getCollisionRobotLinks()
{
    std::vector<std::string> tmp;
    for (auto object : objects_)
    {
        std::shared_ptr<KinematicElement> element = kinematic_elements_[reinterpret_cast<long>(object.second->getUserPointer())].lock();
        if (element->ClosestRobotLink.lock())
        {
            tmp.push_back(element->Segment.getName());
        }
    }
    return tmp;
}
}
