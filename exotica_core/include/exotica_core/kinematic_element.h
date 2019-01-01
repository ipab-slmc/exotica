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
    KinematicElement(int id, std::shared_ptr<KinematicElement> parent, const KDL::Segment& segment) : Parent(parent), Segment(segment), Id(id)
    {
    }

    ~KinematicElement()
    {
        // Remove from parent to avoid expired pointers
        std::shared_ptr<KinematicElement> myParent = Parent.lock();
        if (myParent)
        {
            myParent->removeExpiredChildren();
        }
    }

    inline void updateClosestRobotLink()
    {
        std::shared_ptr<KinematicElement> element = Parent.lock();
        ClosestRobotLink = std::shared_ptr<KinematicElement>(nullptr);
        while (element && element->Id > 0)
        {
            if (element->isRobotLink)
            {
                ClosestRobotLink = element;
                break;
            }
            element = element->Parent.lock();
        }
        setChildrenClosestRobotLink();
    }

    inline KDL::Frame getPose(const double& x = 0.0)
    {
        if (IsTrajectoryGenerated)
        {
            return GeneratedOffset;
        }
        else
        {
            return Segment.pose(x);
        }
    }

    int Id;
    int ControlId = -1;
    bool IsControlled = false;
    std::weak_ptr<KinematicElement> Parent;
    std::string ParentName;
    std::vector<std::weak_ptr<KinematicElement>> Children;
    std::weak_ptr<KinematicElement> ClosestRobotLink = std::shared_ptr<KinematicElement>(nullptr);
    KDL::Segment Segment = KDL::Segment();
    KDL::Frame Frame = KDL::Frame::Identity();
    KDL::Frame GeneratedOffset = KDL::Frame::Identity();
    bool IsTrajectoryGenerated = false;
    std::vector<double> JointLimits;
    shapes::ShapeConstPtr Shape = nullptr;
    std::string ShapeResourcePath = "";
    Eigen::Vector3d Scale = Eigen::Vector3d::Ones();
    bool isRobotLink = false;
    Eigen::Vector4d Color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);

    inline void removeExpiredChildren()
    {
        for (size_t i = 0; i < Children.size(); i++)
        {
            if (Children[i].expired())
            {
                Children.erase(Children.begin() + i);
            }
        }
    }

private:
    inline void setChildrenClosestRobotLink()
    {
        std::stack<std::shared_ptr<KinematicElement>> elements;
        for (auto child : Children) elements.push(child.lock());
        while (!elements.empty())
        {
            auto parent = elements.top();
            elements.pop();
            parent->ClosestRobotLink = ClosestRobotLink;
            for (auto child : parent->Children) elements.push(child.lock());
        }
    }
};

#endif  // KINEMATICELEMENT_H
