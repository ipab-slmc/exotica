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

#ifndef EXOTICA_CORE_KINEMATIC_ELEMENT_H_
#define EXOTICA_CORE_KINEMATIC_ELEMENT_H_

#include <exotica_core/tools.h>
#include <geometric_shapes/shapes.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <stack>

namespace exotica
{
class VisualElement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string name;
    shapes::ShapePtr shape = nullptr;
    std::string shape_resource_path = "";
    KDL::Frame frame = KDL::Frame::Identity();
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    Eigen::Vector4d color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
};

class KinematicElement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KinematicElement(int _id, std::shared_ptr<KinematicElement> _parent, const KDL::Segment& _segment) : id(_id), parent(_parent), segment(_segment)
    {
    }

    ~KinematicElement()
    {
        // Remove from parent to avoid expired pointers
        std::shared_ptr<KinematicElement> my_parent = parent.lock();
        if (my_parent)
        {
            my_parent->RemoveExpiredChildren();
        }
    }

    inline void UpdateClosestRobotLink()
    {
        std::shared_ptr<KinematicElement> element = parent.lock();
        closest_robot_link = std::shared_ptr<KinematicElement>(nullptr);
        while (element && element->id > 0)
        {
            if (element->is_robot_link)
            {
                closest_robot_link = element;
                break;
            }
            element = element->parent.lock();
        }
        SetChildrenClosestRobotLink();
    }

    inline KDL::Frame GetPose(const double& x = 0.0)
    {
        if (is_trajectory_generated)
        {
            return generated_offset;
        }
        else
        {
            return segment.pose(x);
        }
    }

    inline void RemoveExpiredChildren()
    {
        for (size_t i = 0; i < children.size(); ++i)
        {
            if (children[i].expired())
            {
                children.erase(children.begin() + i);
            }
        }
    }

    int id;
    int control_id = -1;
    bool is_controlled = false;
    std::weak_ptr<KinematicElement> parent;
    std::string parent_name;
    std::vector<std::weak_ptr<KinematicElement>> children;
    std::weak_ptr<KinematicElement> closest_robot_link = std::shared_ptr<KinematicElement>(nullptr);
    KDL::Segment segment = KDL::Segment();
    KDL::Frame frame = KDL::Frame::Identity();
    KDL::Frame generated_offset = KDL::Frame::Identity();
    bool is_trajectory_generated = false;
    std::vector<double> joint_limits;
    double velocity_limit = std::numeric_limits<double>::quiet_NaN();
    double acceleration_limit = std::numeric_limits<double>::quiet_NaN();
    shapes::ShapeConstPtr shape = nullptr;
    std::string shape_resource_path = std::string();
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    bool is_robot_link = false;
    Eigen::Vector4d color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
    std::vector<VisualElement> visual;

private:
    inline void SetChildrenClosestRobotLink()
    {
        std::stack<std::shared_ptr<KinematicElement>> elements;
        for (auto child : children) elements.push(child.lock());
        while (!elements.empty())
        {
            auto parent = elements.top();
            elements.pop();
            parent->closest_robot_link = closest_robot_link;
            for (auto child : parent->children) elements.push(child.lock());
        }
    }
};
}  // namespace exotica

#endif  // EXOTICA_CORE_KINEMATIC_ELEMENT_H_
