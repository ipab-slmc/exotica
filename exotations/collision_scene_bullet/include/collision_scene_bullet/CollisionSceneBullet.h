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

#ifndef CollisionSceneBullet_H
#define CollisionSceneBullet_H

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <btBulletCollisionCommon.h>

#include <eigen_conversions/eigen_kdl.h>
#include <exotica/CollisionScene.h>
#include <exotica/Tools/Conversions.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

namespace exotica
{
class CollisionSceneBullet : public CollisionScene
{
public:
    struct CollisionData
    {
        CollisionData(CollisionSceneBullet* scene) : Scene(scene), Self(true) {}
        // fcl::CollisionRequestd Request;
        // fcl::CollisionResultd Result;
        CollisionSceneBullet* Scene;
        bool Self;
        double SafeDistance;
    };

    struct DistanceData
    {
        DistanceData(CollisionSceneBullet* scene) : Scene(scene), Self(true), Distance(1e300) {}
        // fcl::DistanceRequestd Request;
        // fcl::DistanceResultd Result;
        CollisionSceneBullet* Scene;
        std::vector<CollisionProxy> Proxies;
        double Distance;
        bool Self;
    };

    CollisionSceneBullet();

    /**
       * \brief Destructor
       */
    virtual ~CollisionSceneBullet();

    // static bool isAllowedToCollide(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, bool self, CollisionSceneBullet* scene);
    // static bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);
    // static bool collisionCallbackDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& dist);

    /**
       * \brief Check if the whole robot is valid (collision only).
       * @param self Indicate if self collision check is required.
       * @return True, if the state is collision free.
       */
    virtual bool isStateValid(bool self = true, double safe_distance = 0.0);
    virtual bool isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance = 0.0);

    ///
    /// \brief Computes collision distances.
    /// \param self Indicate if self collision check is required.
    /// \return Collision proximity objects for all colliding pairs of objects.
    ///
    virtual std::vector<CollisionProxy> getCollisionDistance(bool self);
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const std::string& o2);
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const bool& self = true);
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::vector<std::string>& objects, const bool& self = true);

    /**
       * @brief      Gets the collision world links.
       *
       * @return     The collision world links.
       */
    virtual std::vector<std::string> getCollisionWorldLinks();

    /**
       * @brief      Gets the collision robot links.
       *
       * @return     The collision robot links.
       */
    virtual std::vector<std::string> getCollisionRobotLinks();

    virtual Eigen::Vector3d getTranslation(const std::string& name);

    ///
    /// \brief Creates the collision scene from kinematic elements.
    /// \param objects Vector kinematic element pointers of collision objects.
    ///
    virtual void updateCollisionObjects(const std::map<std::string, std::shared_ptr<KinematicElement>>& objects);

    ///
    /// \brief Updates collision object transformations from the kinematic tree.
    ///
    virtual void updateCollisionObjectTransforms();

private:
    btDefaultCollisionConfiguration bt_collision_configuration_;
    btDbvtBroadphase bt_collision_broadphase_;
    // OverlapFilterCallback filter_callback_;
    std::shared_ptr<btCollisionDispatcher> bt_collision_dispatcher_;
    std::shared_ptr<btCollisionWorld> bt_collision_world_;

    std::unique_ptr<btCollisionShape> constructBulletCollisionObject(std::shared_ptr<KinematicElement> element);
    // static void checkCollision(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, CollisionData* data);
    // static void computeDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, DistanceData* data);

    virtual std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const bool& self = true, const bool& disableCollisionSceneUpdate = false);

    std::unordered_map<std::string, std::shared_ptr<btCollisionObject>> bullet_collision_objects_;
    std::vector<std::unique_ptr<btCollisionShape>> bullet_collision_shapes_;
    std::vector<std::shared_ptr<KinematicElement>> kinematic_elements_;
};

typedef std::shared_ptr<CollisionSceneBullet> CollisionSceneBullet_ptr;
}

#endif  // CollisionSceneBullet_H
