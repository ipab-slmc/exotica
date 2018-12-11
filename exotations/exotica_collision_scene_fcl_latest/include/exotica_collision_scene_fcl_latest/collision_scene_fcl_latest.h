/*
 *      Author: Vladimir Ivan, Wolfgang Merkt
 *
 * Copyright (c) 2017, University of Edinburgh
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

#ifndef EXOTICA_COLLISION_SCENE_FCL_LATEST_COLLISION_SCENE_FCL_LATEST_H_
#define EXOTICA_COLLISION_SCENE_FCL_LATEST_COLLISION_SCENE_FCL_LATEST_H_

#include <iostream>

#include <exotica/CollisionScene.h>
#include <exotica/Tools/Conversions.h>

#include <fcl/fcl.h>  // FCL 0.6 as provided by fcl_catkin

#include <eigen_conversions/eigen_kdl.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

namespace exotica
{
class CollisionSceneFCLLatest : public CollisionScene
{
public:
    struct CollisionData
    {
        CollisionData(CollisionSceneFCLLatest* scene) : scene(scene) {}
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        CollisionSceneFCLLatest* scene;
        bool self = true;
        double safe_distance;
    };

    struct DistanceData
    {
        DistanceData(CollisionSceneFCLLatest* scene) : scene(scene) {}
        fcl::DistanceRequestd request;
        fcl::DistanceResultd result;
        CollisionSceneFCLLatest* scene;
        std::vector<CollisionProxy> proxies;
        double Distance = 1e300;
        bool self = true;
    };

    CollisionSceneFCLLatest();
    virtual ~CollisionSceneFCLLatest();

    void setup() override;

    bool isAllowedToCollide(const std::string& o1, const std::string& o2, const bool& self) override;

    static bool IsAllowedToCollide(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, bool self, CollisionSceneFCLLatest* scene);
    static bool CollisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);
    static bool CollisionCallbackDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& dist);

    /**
       * \brief Check if the whole robot is valid (collision only).
       * @param self Indicate if self collision check is required.
       * @return True, if the state is collision free.
       */
    bool isStateValid(bool self = true, double safe_distance = 0.0) override;
    bool isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance = 0.0) override;

    ///
    /// \brief Computes collision distances.
    /// \param self Indicate if self collision check is required.
    /// \return Collision proximity objects for all colliding pairs of objects.
    ///
    std::vector<CollisionProxy> getCollisionDistance(bool self) override;
    std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const std::string& o2) override;
    std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const bool& self = true) override;
    std::vector<CollisionProxy> getCollisionDistance(const std::vector<std::string>& objects, const bool& self = true) override;
    std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const bool& self = true, const bool& disableCollisionSceneUpdate = false) override;

    /**
     * @brief      Performs a continuous collision check between two objects with a linear interpolation between two given 
     *
     * @param[in]  o1       The first collision object, by name.
     * @param[in]  tf1_beg  The beginning transform for o1.
     * @param[in]  tf1_end  The end transform for o1.
     * @param[in]  o2       The second collision object, by name.
     * @param[in]  tf2_beg  The beginning transform for o2.
     * @param[in]  tf2_end  The end transform for o2.
     *
     * @return     ContinuousCollisionProxy.
     */
    ContinuousCollisionProxy continuousCollisionCheck(const std::string& o1, const KDL::Frame& tf1_beg, const KDL::Frame& tf1_end, const std::string& o2, const KDL::Frame& tf2_beg, const KDL::Frame& tf2_end) override;

    /**
       * @brief      Gets the collision world links.
       *
       * @return     The collision world links.
       */
    std::vector<std::string> getCollisionWorldLinks() override;

    /**
      * @brief      Gets the KinematicElements associated with the collision world links.
      * @return     The KinematicElements associated with the collision world links.
      */
    std::vector<std::shared_ptr<KinematicElement>> getCollisionWorldLinkElements() override;

    /**
       * @brief      Gets the collision robot links.
       *
       * @return     The collision robot links.
       */
    std::vector<std::string> getCollisionRobotLinks() override;

    Eigen::Vector3d getTranslation(const std::string& name) override;

    ///
    /// \brief Creates the collision scene from kinematic elements.
    /// \param objects Vector kinematic element pointers of collision objects.
    ///
    void updateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects) override;

    ///
    /// \brief Updates collision object transformations from the kinematic tree.
    ///
    void updateCollisionObjectTransforms() override;

private:
    std::shared_ptr<fcl::CollisionObjectd> ConstructFclCollisionObject(long i, std::shared_ptr<KinematicElement> element);
    static void CheckCollision(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, CollisionData* data);
    static void ComputeDistance(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, DistanceData* data);

    std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> fcl_cache_;
    std::vector<fcl::CollisionObjectd*> fcl_objects_;
    std::vector<std::weak_ptr<KinematicElement>> kinematic_elements_;
};
}

#endif  // EXOTICA_COLLISION_SCENE_FCL_LATEST_COLLISION_SCENE_FCL_LATEST_H_
