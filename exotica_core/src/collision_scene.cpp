//
// Copyright (c) 2020, University of Oxford
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

#include <exotica_core/collision_scene.h>
#include <exotica_core/scene.h>

#include <exotica_core/collision_scene_initializer.h>

namespace exotica
{
inline bool IsRobotLink(std::shared_ptr<KinematicElement> e)
{
    return e->is_robot_link || e->closest_robot_link.lock();
}

void CollisionScene::InstantiateBase(const Initializer& init)
{
    Object::InstantiateObject(init);
    CollisionSceneInitializer collision_scene_initializer = CollisionSceneInitializer(init);

    this->SetReplacePrimitiveShapesWithMeshes(collision_scene_initializer.ReplacePrimitiveShapesWithMeshes);
    this->set_replace_cylinders_with_capsules(collision_scene_initializer.ReplaceCylindersWithCapsules);
    this->SetWorldLinkPadding(collision_scene_initializer.WorldLinkPadding);
    this->SetRobotLinkPadding(collision_scene_initializer.RobotLinkPadding);
    this->SetWorldLinkScale(collision_scene_initializer.WorldLinkScale);
    this->SetRobotLinkScale(collision_scene_initializer.RobotLinkScale);
    this->robot_link_replacement_config_ = collision_scene_initializer.RobotLinkReplacementConfig;

    if (debug_) INFO_NAMED(object_name_, "Initialized CollisionScene of type " << GetObjectName());
}

bool CollisionScene::IsAllowedToCollide(const std::string& o1, const std::string& o2, const bool& self)
{
    std::shared_ptr<KinematicElement> e1 = scene_.lock()->GetKinematicTree().FindKinematicElementByName(o1);
    std::shared_ptr<KinematicElement> e2 = scene_.lock()->GetKinematicTree().FindKinematicElementByName(o2);

    bool isRobot1 = IsRobotLink(e1);
    bool isRobot2 = IsRobotLink(e2);
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

}  // namespace exotica
