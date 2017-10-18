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

#ifndef COLLISIONSCENEFCL_H
#define COLLISIONSCENEFCL_H

#include <exotica/CollisionScene.h>
#include <exotica/Tools/Conversions.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/octree.h>
#include <fcl/shape/geometric_shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

namespace exotica
{
class CollisionSceneFCL : public CollisionScene
{
public:
    struct CollisionData
    {
        CollisionData(CollisionSceneFCL* scene) : Scene(scene), Self(true) {}
        fcl::CollisionRequest Request;
        fcl::CollisionResult Result;
        CollisionSceneFCL* Scene;
        bool Self;
        double SafeDistance;
    };

    CollisionSceneFCL();

    /**
       * \brief Destructor
       */
    virtual ~CollisionSceneFCL();

    static bool isAllowedToCollide(fcl::CollisionObject* o1, fcl::CollisionObject* o2, bool self, CollisionSceneFCL* scene);
    static bool collisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data);
    static bool collisionCallbackDistance(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& dist);
    static bool collisionCallbackContacts(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& dist);

    /**
       * \brief Check if the whole robot is valid (collision only).
       * @param self Indicate if self collision check is required.
       * @return True, if the state is collision free..
       */
    virtual bool isStateValid(bool self = true, double safe_distance = 0.0);
    virtual bool isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance = 0.0);

    /**
       * @brief      Gets the collision world links.
       * @return     The collision world links.
       */
    virtual std::vector<std::string> getCollisionWorldLinks();

    /**
       * @brief      Gets the collision robot links.
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
    std::shared_ptr<fcl::CollisionObject> constructFclCollisionObject(int i, std::shared_ptr<KinematicElement> element);
    static void checkCollision(fcl::CollisionObject* o1, fcl::CollisionObject* o2, CollisionData* data);

    std::map<std::string, std::shared_ptr<fcl::CollisionObject>> fcl_cache_;

    std::vector<fcl::CollisionObject*> fcl_objects_;
    std::vector<std::shared_ptr<KinematicElement>> kinematic_elements_;
};

typedef std::shared_ptr<CollisionSceneFCL> CollisionSceneFCL_ptr;
}

#endif  // COLLISIONSCENEFCL_H
