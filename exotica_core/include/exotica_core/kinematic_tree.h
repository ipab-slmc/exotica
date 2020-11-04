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

#ifndef EXOTICA_CORE_KINEMATIC_TREE_H_
#define EXOTICA_CORE_KINEMATIC_TREE_H_

#include <map>
#include <random>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>

#include <exotica_core/kinematic_element.h>

namespace exotica
{
enum BaseType
{
    FIXED = 0,
    FLOATING = 10,
    PLANAR = 20
};

enum KinematicRequestFlags
{
    KIN_FK = 0,
    KIN_J = 2,
    KIN_FK_VEL = 4,
    KIN_H = 8
};

enum JointLimitType
{
    LIMIT_POSITION_LOWER = 0,
    LIMIT_POSITION_UPPER = 1
};

constexpr double inf = std::numeric_limits<double>::infinity();
constexpr double pi = std::atan(1) * 4;

inline KinematicRequestFlags operator|(KinematicRequestFlags a, KinematicRequestFlags b)
{
    return static_cast<KinematicRequestFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline KinematicRequestFlags operator&(KinematicRequestFlags a, KinematicRequestFlags b)
{
    return static_cast<KinematicRequestFlags>(static_cast<int>(a) & static_cast<int>(b));
}

struct KinematicFrameRequest
{
    KinematicFrameRequest();
    KinematicFrameRequest(std::string _frame_A_link_name, KDL::Frame _frame_A_offset = KDL::Frame(), std::string _frame_B_link_name = "", KDL::Frame _frame_B_offset = KDL::Frame());
    std::string frame_A_link_name;
    KDL::Frame frame_A_offset;
    std::string frame_B_link_name;
    KDL::Frame frame_B_offset;
};

struct KinematicsRequest
{
    KinematicsRequest();
    KinematicRequestFlags flags = KinematicRequestFlags::KIN_FK;
    std::vector<KinematicFrameRequest> frames;  // The segments to which the end-effectors are attached
};

struct KinematicFrame
{
    std::weak_ptr<KinematicElement> frame_A;
    KDL::Frame frame_A_offset;
    std::weak_ptr<KinematicElement> frame_B;
    KDL::Frame frame_B_offset;
    KDL::Frame temp_AB;
    KDL::Frame temp_A;
    KDL::Frame temp_B;
};

/// @brief The KinematicResponse is the container to keep kinematic update data.
/// The corresponding KinematicSolution is created from and indexes into a KinematicResponse.
struct KinematicResponse
{
    KinematicResponse();
    KinematicResponse(KinematicRequestFlags _flags, int _size, int _N = 0);
    KinematicRequestFlags flags = KinematicRequestFlags::KIN_FK;
    std::vector<KinematicFrame> frame;
    Eigen::VectorXd x;
    ArrayFrame Phi;
    ArrayTwist Phi_dot;
    ArrayJacobian jacobian;
    ArrayHessian hessian;
};

/// @brief The KinematicSolution is created from - and maps into - a KinematicResponse.
class KinematicSolution
{
public:
    KinematicSolution();
    KinematicSolution(int start, int length);
    void Create(std::shared_ptr<KinematicResponse> solution);
    int start = -1;
    int length = -1;
    Eigen::Map<Eigen::VectorXd> X{nullptr, 0};
    Eigen::Map<ArrayFrame> Phi{nullptr, 0};
    Eigen::Map<ArrayTwist> Phi_dot{nullptr, 0};
    Eigen::Map<ArrayJacobian> jacobian{nullptr, 0};
    Eigen::Map<ArrayHessian> hessian{nullptr, 0};
};

class KinematicTree : public Uncopyable
{
public:
    void Instantiate(const std::string& joint_group, robot_model::RobotModelPtr model, const std::string& name);
    const std::string& GetRootFrameName() const;
    const std::string& GetRootJointName() const;
    robot_model::RobotModelPtr GetRobotModel() const;
    BaseType GetModelBaseType() const;
    BaseType GetControlledBaseType() const;
    std::shared_ptr<KinematicResponse> RequestFrames(const KinematicsRequest& request);
    void Update(Eigen::VectorXdRefConst x);
    void ResetJointLimits();
    const Eigen::MatrixXd& GetJointLimits() const { return joint_limits_; }
    void SetJointLimitsLower(Eigen::VectorXdRefConst lower_in);
    void SetJointLimitsUpper(Eigen::VectorXdRefConst upper_in);
    void SetJointVelocityLimits(Eigen::VectorXdRefConst velocity_in);
    void SetJointAccelerationLimits(Eigen::VectorXdRefConst acceleration_in);
    void SetFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double>& lower, const std::vector<double>& upper);
    void SetFloatingBaseLimitsPosXYZEulerZYX(const std::vector<double>& lower, const std::vector<double>& upper, const std::vector<double>& velocity, const std::vector<double>& acceleration);
    void SetPlanarBaseLimitsPosXYEulerZ(const std::vector<double>& lower, const std::vector<double>& upper);
    void SetPlanarBaseLimitsPosXYEulerZ(const std::vector<double>& lower, const std::vector<double>& upper, const std::vector<double>& velocity, const std::vector<double>& acceleration);
    std::map<std::string, std::vector<double>> GetUsedJointLimits() const;
    const bool& HasAccelerationLimits() const { return has_acceleration_limit_; }
    const Eigen::VectorXd& GetAccelerationLimits() const { return acceleration_limits_; }
    const Eigen::VectorXd& GetVelocityLimits() const { return velocity_limits_; }
    int GetNumControlledJoints() const;
    int GetNumModelJoints() const;
    void PublishFrames(const std::string& tf_prefix = "exotica");

    const std::vector<std::string>& GetControlledJointNames() const
    {
        return controlled_joints_names_;
    }

    const std::vector<std::string>& GetControlledLinkNames() const
    {
        return controlled_link_names_;
    }

    const std::vector<std::string>& GetModelJointNames() const
    {
        return model_joints_names_;
    }

    const std::vector<std::string>& GetModelLinkNames() const
    {
        return model_link_names_;
    }

    bool HasModelLink(const std::string& link) const;

    Eigen::VectorXd GetControlledLinkMass() const;

    KDL::Frame FK(KinematicFrame& frame) const;
    KDL::Frame FK(std::shared_ptr<KinematicElement> element_A, const KDL::Frame& offset_a, std::shared_ptr<KinematicElement> element_B, const KDL::Frame& offset_b) const;
    KDL::Frame FK(const std::string& element_A, const KDL::Frame& offset_a, const std::string& element_B, const KDL::Frame& offset_b) const;
    Eigen::MatrixXd Jacobian(std::shared_ptr<KinematicElement> element_A, const KDL::Frame& offset_a, std::shared_ptr<KinematicElement> element_B, const KDL::Frame& offset_b) const;
    Eigen::MatrixXd Jacobian(const std::string& element_A, const KDL::Frame& offset_a, const std::string& element_B, const KDL::Frame& offset_b) const;
    exotica::Hessian Hessian(std::shared_ptr<KinematicElement> element_A, const KDL::Frame& offset_a, std::shared_ptr<KinematicElement> element_B, const KDL::Frame& offset_b) const;
    exotica::Hessian Hessian(const std::string& element_A, const KDL::Frame& offset_a, const std::string& element_B, const KDL::Frame& offset_b) const;

    void ResetModel();
    std::shared_ptr<KinematicElement> AddElement(const std::string& name, const Eigen::Isometry3d& transform, const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const std::vector<VisualElement>& visual = {}, bool is_controlled = false);
    std::shared_ptr<KinematicElement> AddEnvironmentElement(const std::string& name, const Eigen::Isometry3d& transform, const std::string& parent = "", shapes::ShapeConstPtr shape = shapes::ShapeConstPtr(nullptr), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const std::vector<VisualElement>& visual = {}, bool is_controlled = false);
    std::shared_ptr<KinematicElement> AddElement(const std::string& name, const Eigen::Isometry3d& transform, const std::string& parent, const std::string& shape_resource_path, Eigen::Vector3d scale = Eigen::Vector3d::Ones(), const KDL::RigidBodyInertia& inertia = KDL::RigidBodyInertia::Zero(), const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0), const std::vector<VisualElement>& visual = {}, bool is_controlled = false);
    void UpdateModel();
    void ChangeParent(const std::string& name, const std::string& parent, const KDL::Frame& pose, bool relative);
    int IsControlled(std::shared_ptr<KinematicElement> joint);
    int IsControlledLink(const std::string& link_name);

    Eigen::VectorXd GetModelState() const;
    std::map<std::string, double> GetModelStateMap() const;

    /// @brief GetKinematicChain get list of joints in a kinematic chain
    /// @param begin link name from which the chain starts
    /// @param end link name at which the chain ends
    /// @return list joints between begin and end
    std::vector<std::string> GetKinematicChain(const std::string& begin, const std::string& end) const;

    /// @brief GetKinematicChainLinks get the links between begin and end of kinematic chain
    /// @param begin link name from which the chain starts
    /// @param end link name at which the chain ends
    /// @return list link between begin and end
    std::vector<std::string> GetKinematicChainLinks(const std::string& begin, const std::string& end) const;

    void SetModelState(Eigen::VectorXdRefConst x);
    void SetModelState(const std::map<std::string, double>& x);
    Eigen::VectorXd GetControlledState() const;

    const std::vector<std::weak_ptr<KinematicElement>>& GetTree() const { return tree_; }
    const std::vector<std::shared_ptr<KinematicElement>>& GetModelTree() const { return model_tree_; }
    const std::map<std::string, std::weak_ptr<KinematicElement>>& GetTreeMap() const { return tree_map_; }
    const std::map<std::string, std::weak_ptr<KinematicElement>>& GetCollisionTreeMap() const { return collision_tree_map_; }
    bool DoesLinkWithNameExist(std::string name) const;  //!< Checks whether a link with this name exists in any of the trees
    std::shared_ptr<KinematicElement> FindKinematicElementByName(const std::string& frame_name);

    const std::vector<std::weak_ptr<KinematicElement>>& GetControlledJoints() const { return controlled_joints_; }
    const std::map<std::string, std::weak_ptr<KinematicElement>>& GetControlledJointsMap() const { return controlled_joints_map_; }
    const std::map<std::string, std::weak_ptr<KinematicElement>>& GetModelJointsMap() const { return model_joints_map_; }
    std::map<std::string, shapes::ShapeType> GetCollisionObjectTypes() const;

    /// @brief SetSeed sets the seed of the random generator for deterministic joint state sampling
    /// @param seed unsigned integer
    void SetSeed(const uint_fast32_t seed) { generator_.seed(seed); }
    /// Random state generation
    Eigen::VectorXd GetRandomControlledState();

    void SetKinematicResponse(std::shared_ptr<KinematicResponse> response_in) { solution_ = response_in; }
    std::shared_ptr<KinematicResponse> GetKinematicResponse() { return solution_; }
    bool debug = false;

private:
    void BuildTree(const KDL::Tree& RobotKinematics);
    void AddElementFromSegmentMapIterator(KDL::SegmentMap::const_iterator segment, std::shared_ptr<KinematicElement> parent);
    void UpdateTree();
    void UpdateFK();
    void UpdateJ();
    void ComputeJ(KinematicFrame& frame, KDL::Jacobian& jacobian) const;
    void UpdateH();
    void ComputeH(KinematicFrame& frame, const KDL::Jacobian& jacobian, exotica::Hessian& hessian) const;

    // Joint limits
    // TODO: Add effort limits
    Eigen::MatrixXd joint_limits_;
    Eigen::VectorXd velocity_limits_;
    Eigen::VectorXd acceleration_limits_;
    bool has_acceleration_limit_ = false;
    void UpdateJointLimits();

    // Random state generation
    std::random_device rd_;
    std::mt19937 generator_;
    std::vector<std::uniform_real_distribution<double>> random_state_distributions_;

    BaseType model_base_type_;
    BaseType controlled_base_type_ = BaseType::FIXED;
    int num_controlled_joints_;  //!< Number of controlled joints in the joint group.
    int num_joints_;             //!< Number of joints of the model (including floating/planar base, passive joints, and uncontrolled joints).
    int state_size_ = -1;
    Eigen::VectorXd tree_state_;
    robot_model::RobotModelPtr model_;
    std::string root_joint_name_ = "";
    std::vector<std::weak_ptr<KinematicElement>> tree_;
    std::vector<std::shared_ptr<KinematicElement>> model_tree_;
    std::vector<std::shared_ptr<KinematicElement>> environment_tree_;
    std::map<std::string, std::weak_ptr<KinematicElement>> tree_map_;
    std::map<std::string, std::weak_ptr<KinematicElement>> collision_tree_map_;
    std::shared_ptr<KinematicElement> root_;
    std::vector<std::weak_ptr<KinematicElement>> controlled_joints_;
    std::map<std::string, std::weak_ptr<KinematicElement>> controlled_joints_map_;
    std::map<std::string, std::weak_ptr<KinematicElement>> model_joints_map_;
    std::vector<std::string> model_joints_names_;
    std::vector<std::string> controlled_joints_names_;
    std::vector<std::string> model_link_names_;
    std::vector<std::string> controlled_link_names_;
    std::shared_ptr<KinematicResponse> solution_ = std::make_shared<KinematicResponse>();
    KinematicRequestFlags flags_;

    std::vector<tf::StampedTransform> debug_tree_;
    std::vector<tf::StampedTransform> debug_frames_;
    ros::Publisher shapes_pub_;
    ros::Publisher octomap_pub_;
    bool debug_scene_changed_;
    visualization_msgs::MarkerArray marker_array_msg_;
    std::string name_;
};
}

#endif  // EXOTICA_CORE_KINEMATIC_TREE_H_
