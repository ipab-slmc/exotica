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

#ifndef EXOTICA_COLLISION_SCENE_FCL_COLLISION_SCENE_FCL_H_
#define EXOTICA_COLLISION_SCENE_FCL_COLLISION_SCENE_FCL_H_

#include <exotica_core/collision_scene.h>
#include <exotica_core/tools/conversions.h>

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
        CollisionData(CollisionSceneFCL* scene_in) : scene(scene_in) {}
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        CollisionSceneFCL* scene;
        bool self = true;
        double safe_distance;
    };

    CollisionSceneFCL();
    virtual ~CollisionSceneFCL();

    void Setup() override;

    static bool IsAllowedToCollide(fcl::CollisionObject* o1, fcl::CollisionObject* o2, bool self, CollisionSceneFCL* scene);
    static bool CollisionCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data);

    /// \brief Check if the whole robot is valid (collision only).
    /// @param self Indicate if self collision check is required.
    /// @return True, if the state is collision free..
    bool IsStateValid(bool self = true, double safe_distance = 0.0) override;
    bool IsCollisionFree(const std::string& o1, const std::string& o2, double safe_distance = 0.0) override;

    /// @brief      Gets the collision world links.
    /// @return     The collision world links.
    std::vector<std::string> GetCollisionWorldLinks() override;

    /// @brief      Gets the KinematicElements associated with the collision world links.
    /// @return     The KinematicElements associated with the collision world links.
    std::vector<std::shared_ptr<KinematicElement>> GetCollisionWorldLinkElements() override;

    /// @brief      Gets the collision robot links.
    /// @return     The collision robot links.
    std::vector<std::string> GetCollisionRobotLinks() override;

    Eigen::Vector3d GetTranslation(const std::string& name) override;

    /// \brief Creates the collision scene from kinematic elements.
    /// \param objects Vector kinematic element pointers of collision objects.
    void UpdateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects) override;

    /// \brief Updates collision object transformations from the kinematic tree.
    void UpdateCollisionObjectTransforms() override;

private:
    std::shared_ptr<fcl::CollisionObject> ConstructFclCollisionObject(long i, std::shared_ptr<KinematicElement> element);
    static void CheckCollision(fcl::CollisionObject* o1, fcl::CollisionObject* o2, CollisionData* data);

    std::map<std::string, std::shared_ptr<fcl::CollisionObject>> fcl_cache_;

    std::vector<fcl::CollisionObject*> fcl_objects_;
    std::vector<std::weak_ptr<KinematicElement>> kinematic_elements_;
};
}

#endif  // EXOTICA_COLLISION_SCENE_FCL_COLLISION_SCENE_FCL_H_
