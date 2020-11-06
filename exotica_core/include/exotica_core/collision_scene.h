//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#ifndef EXOTICA_CORE_COLLISION_SCENE_H_
#define EXOTICA_CORE_COLLISION_SCENE_H_

#include <Eigen/Dense>

#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <exotica_core/factory.h>
#include <exotica_core/kinematic_element.h>
#include <exotica_core/object.h>

#define REGISTER_COLLISION_SCENE_TYPE(TYPE, DERIV) EXOTICA_CORE_REGISTER(exotica::CollisionScene, TYPE, DERIV)
namespace exotica
{
class Scene;

class AllowedCollisionMatrix
{
public:
    AllowedCollisionMatrix() {}
    ~AllowedCollisionMatrix() {}
    AllowedCollisionMatrix(const AllowedCollisionMatrix& acm) : entries_(acm.entries_) {}
    AllowedCollisionMatrix& operator=(const AllowedCollisionMatrix& other)
    {
        if (this != &other)
        {
            entries_ = other.entries_;
        }
        return *this;
    }
    inline void clear() { entries_.clear(); }
    inline bool hasEntry(const std::string& name) const { return entries_.find(name) == entries_.end(); }
    inline void setEntry(const std::string& name1, const std::string& name2) { entries_[name1].insert(name2); }
    inline void getAllEntryNames(std::vector<std::string>& names) const
    {
        names.clear();
        for (auto& it : entries_)
        {
            names.push_back(it.first);
        }
    }
    inline size_t getNumberOfEntries() const { return entries_.size(); }
    inline bool getAllowedCollision(const std::string& name1, const std::string& name2) const
    {
        auto it = entries_.find(name1);
        if (it == entries_.end()) return true;
        return it->second.find(name2) == it->second.end();
    }

private:
    std::unordered_map<std::string, std::unordered_set<std::string>> entries_;
};

struct ContinuousCollisionProxy
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContinuousCollisionProxy() : e1(nullptr), e2(nullptr), in_collision(false), time_of_contact(-1) {}
    std::shared_ptr<KinematicElement> e1;
    std::shared_ptr<KinematicElement> e2;
    KDL::Frame contact_tf1;
    KDL::Frame contact_tf2;
    bool in_collision;
    double time_of_contact;

    // Contact information, filled in if in continuous contact:
    double penetration_depth = 0.0;
    Eigen::Vector3d contact_normal;
    Eigen::Vector3d contact_pos;  // In world frame

    inline std::string Print() const
    {
        std::stringstream ss;
        if (e1 && e2)
        {
            ss << "ContinuousCollisionProxy: '" << e1->segment.getName() << "' - '" << e2->segment.getName() << " in_collision: " << in_collision << " time_of_contact " << time_of_contact << " depth: " << penetration_depth;
        }
        else
        {
            ss << "ContinuousCollisionProxy (empty)";
        }
        return ss.str();
    }
};

struct CollisionProxy
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CollisionProxy() : e1(nullptr), e2(nullptr), distance(0) {}
    std::shared_ptr<KinematicElement> e1;
    std::shared_ptr<KinematicElement> e2;
    Eigen::Vector3d contact1;
    Eigen::Vector3d normal1;
    Eigen::Vector3d contact2;
    Eigen::Vector3d normal2;
    double distance;

    inline std::string Print() const
    {
        std::stringstream ss;
        if (e1 && e2)
        {
            ss << "Proxy: '" << e1->segment.getName() << "' - '" << e2->segment.getName() << "', c1: " << contact1.transpose() << " c2: " << contact2.transpose() << " n1: " << normal1.transpose() << " n2: " << normal2.transpose() << " d: " << distance;
        }
        else
        {
            ss << "Proxy (empty)";
        }
        return ss.str();
    }
};

/// The class of collision scene
class CollisionScene : public Object, public Uncopyable, public virtual InstantiableBase
{
public:
    CollisionScene() {}
    virtual ~CollisionScene() {}
    /// \brief Instantiates the base properties of the CollisionScene
    virtual void InstantiateBase(const Initializer& init);

    /// @brief Setup additional construction that requires initialiser parameter
    virtual void Setup() {}
    /// @brief Returns whether two collision objects/shapes are allowed to collide by name.
    /// @param o1 Name of the frame of the collision object (e.g., base_link_collision_0)
    /// @param o2 Name of the frame of the other collision object (e.g., base_link_collision_0)
    /// @return true The two objects are allowed to collide.
    /// @return false The two objects are excluded, e.g., by an ACM.
    virtual bool IsAllowedToCollide(const std::string& o1, const std::string& o2, const bool& self);

    /// \brief Checks if the whole robot is valid (collision only).
    /// @param self Indicate if self collision check is required.
    /// @return True, if the state is collision free..
    virtual bool IsStateValid(bool self = true, double safe_distance = 0.0) = 0;

    /// @brief Checks if two objects are in collision.
    /// @param o1 Name of object 1.
    /// @param o2 Name of object 2.
    /// @return True is the two objects are not colliding.
    virtual bool IsCollisionFree(const std::string& o1, const std::string& o2, double safe_distance = 0.0) { ThrowPretty("Not implemented!"); }
    /// \brief Computes collision distances.
    /// \param self Indicate if self collision check is required.
    /// \return Collision proximity objects for all colliding pairs of shapes.
    ///
    virtual std::vector<CollisionProxy> GetCollisionDistance(bool self) { ThrowPretty("Not implemented!"); }
    /// \brief Computes collision distances between two objects.
    /// \param o1 Name of object 1.
    /// \param o2 Name of object 2.
    /// \return Vector of proximity objects.
    virtual std::vector<CollisionProxy> GetCollisionDistance(const std::string& o1, const std::string& o2) { ThrowPretty("Not implemented!"); }
    /// @brief Gets the closest distance of any collision object which is allowed to collide with any collision object related to object o1.
    /// @param[in] o1 Name of object 1.
    /// @return Vector of proximity objects.
    virtual std::vector<CollisionProxy> GetCollisionDistance(const std::string& o1, const bool& self) { ThrowPretty("Not implemented!"); }
    /// @brief      Gets the closest distance of any collision object which is allowed to collide with any collision object related to object o1.
    /// @param[in]  o1    Name of object 1.
    /// @param[in]  disable_collision_scene_update    Allows disabling of collision object transforms (requires manual update).
    /// @return     Vector of proximity objects.
    virtual std::vector<CollisionProxy> GetCollisionDistance(const std::string& o1, const bool& self, const bool& disable_collision_scene_update) { ThrowPretty("Not implemented!"); }
    /// @brief      Gets the closest distance of any collision object which is
    /// allowed to collide with any collision object related to any of the objects.
    /// @param[in]  objects    Vector of object names.
    /// @return     Vector of proximity objects.
    virtual std::vector<CollisionProxy> GetCollisionDistance(const std::vector<std::string>& objects, const bool& self) { ThrowPretty("Not implemented!"); }
    /// @brief      Gets the closest distances between links within the robot that are closer than check_margin
    /// @param[in]  check_margin    Margin for distance checks - only objects closer than this margin will be checked
    virtual std::vector<CollisionProxy> GetRobotToRobotCollisionDistance(double check_margin) { ThrowPretty("Not implemented!"); }
    /// @brief      Gets the closest distances between links of the robot and the environment that are closer than check_margin
    /// @param[in]  check_margin    Margin for distance checks - only objects closer than this margin will be checked
    virtual std::vector<CollisionProxy> GetRobotToWorldCollisionDistance(double check_margin) { ThrowPretty("Not implemented!"); }
    /// @brief      Gets the collision world links.
    /// @return     The collision world links.
    virtual std::vector<std::string> GetCollisionWorldLinks() = 0;

    /// @brief      Gets the collision robot links.
    /// @return     The collision robot links.
    virtual std::vector<std::string> GetCollisionRobotLinks() = 0;

    /// @brief      Performs a continuous collision check between two objects with a linear interpolation between two given
    /// @param[in]  o1       The first collision object, by name.
    /// @param[in]  tf1_beg  The beginning transform for o1.
    /// @param[in]  tf1_end  The end transform for o1.
    /// @param[in]  o2       The second collision object, by name.
    /// @param[in]  tf2_beg  The beginning transform for o2.
    /// @param[in]  tf2_end  The end transform for o2.
    /// @return     ContinuousCollisionProxy.
    virtual ContinuousCollisionProxy ContinuousCollisionCheck(const std::string& o1, const KDL::Frame& tf1_beg, const KDL::Frame& tf1_end, const std::string& o2, const KDL::Frame& tf2_beg, const KDL::Frame& tf2_end) { ThrowPretty("Not implemented!"); }
    /// @brief      Performs a continuous collision check by casting the active objects passed in against the static environment.
    /// @param[in]  motion_transforms   A tuple consisting out of collision object name and its beginning and final transform.
    /// @return     Vector of deepest ContinuousCollisionProxy (one per dimension).
    virtual std::vector<ContinuousCollisionProxy> ContinuousCollisionCast(const std::vector<std::vector<std::tuple<std::string, Eigen::Isometry3d, Eigen::Isometry3d>>>& motion_transforms) { ThrowPretty("Not implemented!"); }
    /// @brief      Returns the translation of the named collision object.
    /// @param[in]  name    Name of the collision object to query.
    virtual Eigen::Vector3d GetTranslation(const std::string& name) = 0;

    void SetACM(const AllowedCollisionMatrix& acm)
    {
        acm_ = acm;
    }

    bool GetAlwaysExternallyUpdatedCollisionScene() const { return always_externally_updated_collision_scene_; }
    void SetAlwaysExternallyUpdatedCollisionScene(const bool value)
    {
        always_externally_updated_collision_scene_ = value;
    }

    double GetRobotLinkScale() const { return robot_link_scale_; }
    void SetRobotLinkScale(const double scale)
    {
        if (scale < 0.0)
            ThrowPretty("Link scaling needs to be greater than or equal to 0");
        robot_link_scale_ = scale;
        needs_update_of_collision_objects_ = true;
    }

    double GetWorldLinkScale() const { return world_link_scale_; }
    void SetWorldLinkScale(const double scale)
    {
        if (scale < 0.0)
            ThrowPretty("Link scaling needs to be greater than or equal to 0");
        world_link_scale_ = scale;
        needs_update_of_collision_objects_ = true;
    }

    double GetRobotLinkPadding() const { return robot_link_padding_; }
    void SetRobotLinkPadding(const double padding)
    {
        if (padding < 0.0)
            HIGHLIGHT_NAMED("SetRobotLinkPadding", "Generally, padding should be positive.");
        robot_link_padding_ = padding;
        needs_update_of_collision_objects_ = true;
    }

    double GetWorldLinkPadding() const { return world_link_padding_; }
    void SetWorldLinkPadding(const double padding)
    {
        if (padding < 0.0)
            HIGHLIGHT_NAMED("SetRobotLinkPadding", "Generally, padding should be positive.");
        world_link_padding_ = padding;
        needs_update_of_collision_objects_ = true;
    }

    bool GetReplacePrimitiveShapesWithMeshes() const { return replace_primitive_shapes_with_meshes_; }
    void SetReplacePrimitiveShapesWithMeshes(const bool value)
    {
        replace_primitive_shapes_with_meshes_ = value;
        needs_update_of_collision_objects_ = true;
    }

    /// \brief Creates the collision scene from kinematic elements.
    /// \param objects Vector kinematic element pointers of collision objects.
    virtual void UpdateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects) = 0;

    /// \brief Updates collision object transformations from the kinematic tree.
    virtual void UpdateCollisionObjectTransforms() = 0;

    bool get_replace_cylinders_with_capsules() const { return replace_cylinders_with_capsules_; }
    void set_replace_cylinders_with_capsules(const bool value)
    {
        replace_cylinders_with_capsules_ = value;
        needs_update_of_collision_objects_ = true;
    }

    bool debug_ = false;

    /// \brief Sets a scene pointer to the CollisionScene for access to methods
    void AssignScene(std::shared_ptr<Scene> scene)
    {
        scene_ = scene;
    }

protected:
    /// Indicates whether TriggerUpdateCollisionObjects needs to be called.
    bool needs_update_of_collision_objects_ = true;

    /// Stores a pointer to the Scene which owns this CollisionScene
    std::weak_ptr<Scene> scene_;

    /// The allowed collision matrix
    AllowedCollisionMatrix acm_;

    /// Whether the collision scene is automatically updated - if not, update on queries
    bool always_externally_updated_collision_scene_ = false;

    /// Robot link scaling
    double robot_link_scale_ = 1.0;

    /// World link scaling
    double world_link_scale_ = 1.0;

    /// Robot link padding
    double robot_link_padding_ = 0.0;

    /// World link padding
    double world_link_padding_ = 0.0;

    /// Replace primitive shapes with meshes internally (e.g. when primitive shape algorithms are brittle, i.e. in FCL)
    bool replace_primitive_shapes_with_meshes_ = false;

    /// Replace cylinders with capsules internally
    bool replace_cylinders_with_capsules_ = false;

    /// Filename for config file (YAML) which contains shape replacements
    std::string robot_link_replacement_config_ = "";
};

typedef std::shared_ptr<CollisionScene> CollisionScenePtr;
}

#endif  // EXOTICA_CORE_COLLISION_SCENE_H_
