#ifndef COLLISIONSCENE_H
#define COLLISIONSCENE_H

#include <exotica/Factory.h>
#include <exotica/KinematicElement.h>
#include <exotica/Object.h>
#include <exotica/Tools.h>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#define REGISTER_COLLISION_SCENE_TYPE(TYPE, DERIV) EXOTICA_REGISTER(exotica::CollisionScene, TYPE, DERIV)
namespace exotica
{
class AllowedCollisionMatrix
{
public:
    AllowedCollisionMatrix() {}
    AllowedCollisionMatrix(const AllowedCollisionMatrix& acm) { entries_ = acm.entries_; }
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
    ContinuousCollisionProxy() : e1(nullptr), e2(nullptr), in_collision(false), time_of_contact(-1) {}
    std::shared_ptr<KinematicElement> e1;
    std::shared_ptr<KinematicElement> e2;
    KDL::Frame contact_tf1;
    KDL::Frame contact_tf2;
    bool in_collision;
    double time_of_contact;

    inline std::string print() const
    {
        std::stringstream ss;
        if (e1 && e2)
        {
            ss << "ContinuousCollisionProxy: '" << e1->Segment.getName() << "' - '" << e2->Segment.getName() << " in_collision: " << in_collision << " time_of_contact " << time_of_contact;
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
    CollisionProxy() : e1(nullptr), e2(nullptr), distance(0) {}
    std::shared_ptr<KinematicElement> e1;
    std::shared_ptr<KinematicElement> e2;
    Eigen::Vector3d contact1;
    Eigen::Vector3d normal1;
    Eigen::Vector3d contact2;
    Eigen::Vector3d normal2;
    double distance;

    inline std::string print() const
    {
        std::stringstream ss;
        if (e1 && e2)
        {
            ss << "Proxy: '" << e1->Segment.getName() << "' - '" << e2->Segment.getName() << "', c1: " << contact1.transpose() << " c2: " << contact2.transpose() << " n1: " << normal1.transpose() << " n2: " << normal2.transpose() << " d: " << distance;
        }
        else
        {
            ss << "Proxy (empty)";
        }
        return ss.str();
    }
};

/// The class of collision scene
class CollisionScene : public Uncopyable
{
public:
    CollisionScene() {}
    /**
       * \brief Destructor
       */
    virtual ~CollisionScene() {}
    /**
       * \brief Checks if the whole robot is valid (collision only).
       * @param self Indicate if self collision check is required.
       * @return True, if the state is collision free..
       */
    virtual bool isStateValid(bool self = true, double safe_distance = 0.0) = 0;

    ///
    /// @brief Checks if two objects are in collision.
    /// @param o1 Name of object 1.
    /// @param o2 Name of object 2.
    /// @return True is the two objects are not colliding.
    ///
    virtual bool isCollisionFree(const std::string& o1, const std::string& o2, double safe_distance = 0.0) { throw_pretty("Not implemented!"); }
    ///
    /// \brief Computes collision distances.
    /// \param self Indicate if self collision check is required.
    /// \return Collision proximity objects for all colliding pairs of shapes.
    ///
    virtual std::vector<CollisionProxy> getCollisionDistance(bool self) { throw_pretty("Not implemented!"); }
    ///
    /// \brief Computes collision distances between two objects.
    /// \param o1 Name of object 1.
    /// \param o2 Name of object 2.
    /// \return Vector of proximity objects.
    ///
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const std::string& o2) { throw_pretty("Not implemented!"); }
    /**
   * @brief      Gets the closest distance of any collision object which is
   * allowed to collide with any collision object related to object o1.
   * @param[in]  o1    Name of object 1.
   * @return     Vector of proximity objects.
   */
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const bool& self) { throw_pretty("Not implemented!"); }
    /**
   * @brief      Gets the closest distance of any collision object which is
   * allowed to collide with any collision object related to object o1.
   * @param[in]  o1    Name of object 1.
   * @param[in]  disableCollisionSceneUpdate    Allows disabling of collision object transforms (requires manual update).
   * @return     Vector of proximity objects.
   */
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::string& o1, const bool& self, const bool& disableCollisionSceneUpdate) { throw_pretty("Not implemented!"); }
    /**
   * @brief      Gets the closest distance of any collision object which is
   * allowed to collide with any collision object related to any of the objects.
   * @param[in]  objects    Vector of object names.
   * @return     Vector of proximity objects.
   */
    virtual std::vector<CollisionProxy> getCollisionDistance(const std::vector<std::string>& objects, const bool& self) { throw_pretty("Not implemented!"); }
    /**
       * @brief      Gets the collision world links.
       * @return     The collision world links.
       */
    virtual std::vector<std::string> getCollisionWorldLinks() = 0;

    /**
       * @brief      Gets the collision robot links.
       * @return     The collision robot links.
       */
    virtual std::vector<std::string> getCollisionRobotLinks() = 0;

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
    virtual ContinuousCollisionProxy continuousCollisionCheck(const std::string& o1, const KDL::Frame& tf1_beg, const KDL::Frame& tf1_end, const std::string& o2, const KDL::Frame& tf2_beg, const KDL::Frame& tf2_end) { throw_pretty("Not implemented!"); }
    virtual Eigen::Vector3d getTranslation(const std::string& name) = 0;

    inline void setACM(const AllowedCollisionMatrix& acm)
    {
        acm_ = acm;
    }

    inline bool getAlwaysExternallyUpdatedCollisionScene() { return alwaysExternallyUpdatedCollisionScene_; }
    inline void setAlwaysExternallyUpdatedCollisionScene(const bool& value)
    {
        alwaysExternallyUpdatedCollisionScene_ = value;
    }

    inline double getRobotLinkScale() { return robotLinkScale_; }
    inline void setRobotLinkScale(const double& scale)
    {
        if (scale < 0.0)
            throw_pretty("Link scaling needs to be greater than or equal to 0");
        robotLinkScale_ = scale;
    }

    inline double getWorldLinkScale() { return worldLinkScale_; }
    inline void setWorldLinkScale(const double& scale)
    {
        if (scale < 0.0)
            throw_pretty("Link scaling needs to be greater than or equal to 0");
        worldLinkScale_ = scale;
    }

    inline double getRobotLinkPadding() { return robotLinkPadding_; }
    inline void setRobotLinkPadding(const double& padding)
    {
        if (padding < 0.0)
            HIGHLIGHT_NAMED("setRobotLinkPadding", "Generally, padding should be positive.");
        robotLinkPadding_ = padding;
    }

    inline double getWorldLinkPadding() { return worldLinkPadding_; }
    inline void setWorldLinkPadding(const double& padding)
    {
        if (padding < 0.0)
            HIGHLIGHT_NAMED("setRobotLinkPadding", "Generally, padding should be positive.");
        worldLinkPadding_ = padding;
    }

    inline bool getReplacePrimitiveShapesWithMeshes() { return replacePrimitiveShapesWithMeshes_; }
    inline void setReplacePrimitiveShapesWithMeshes(const bool& value)
    {
        replacePrimitiveShapesWithMeshes_ = value;
    }

    ///
    /// \brief Creates the collision scene from kinematic elements.
    /// \param objects Vector kinematic element pointers of collision objects.
    ///
    virtual void updateCollisionObjects(const std::map<std::string, std::weak_ptr<KinematicElement>>& objects) = 0;

    ///
    /// \brief Updates collision object transformations from the kinematic tree.
    ///
    virtual void updateCollisionObjectTransforms() = 0;

    bool replaceCylindersWithCapsules = false;

protected:
    /// The allowed collision matrix
    AllowedCollisionMatrix acm_;

    /// Whether the collision scene is automatically updated - if not, update on queries
    bool alwaysExternallyUpdatedCollisionScene_ = false;

    /// Robot link scaling
    double robotLinkScale_ = 1.0;

    /// World link scaling
    double worldLinkScale_ = 1.0;

    /// Robot link padding
    double robotLinkPadding_ = 0.0;

    /// World link padding
    double worldLinkPadding_ = 0.0;

    /// Replace primitive shapes with meshes internally (e.g. when primitive shape algorithms are brittle, i.e. in FCL)
    bool replacePrimitiveShapesWithMeshes_ = false;
};

typedef exotica::Factory<exotica::CollisionScene> CollisionScene_fac;
typedef std::shared_ptr<CollisionScene> CollisionScene_ptr;
}

#endif  // COLLISIONSCENE_H
