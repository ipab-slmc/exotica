#ifndef KINEMATICELEMENT_H
#define KINEMATICELEMENT_H

#include <exotica_core/tools.h>
#include <geometric_shapes/shapes.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <stack>

class KinematicElement
{
public:
    KinematicElement(int _id, std::shared_ptr<KinematicElement> _parent, const KDL::Segment& _segment) : parent(_parent), segment(_segment), id(_id)
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
        for (size_t i = 0; i < children.size(); i++)
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
    shapes::ShapeConstPtr shape = nullptr;
    std::string shape_resource_path = "";
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    bool is_robot_link = false;
    Eigen::Vector4d color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);

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

#endif  // KINEMATICELEMENT_H
