#ifndef KINEMATICELEMENT_H
#define KINEMATICELEMENT_H

#include <exotica/Tools.h>
#include <kdl/segment.hpp>
#include <kdl/frames.hpp>
#include <geometric_shapes/shapes.h>

class KinematicElement
{
public:
    KinematicElement(int id, std::shared_ptr<KinematicElement> parent, KDL::Segment segment) :
        Parent(parent), Segment(segment), Id(id), IsControlled(false),
        ControlId(-1), Shape(nullptr), isRobotLink(false), ClosestRobotLink(nullptr)
    {
    }
    inline void updateClosestRobotLink()
    {
        std::shared_ptr<KinematicElement> element = Parent;
        ClosestRobotLink = nullptr;
        while(element && element->Id>0)
        {
            if(element->isRobotLink)
            {
                ClosestRobotLink = element;
                break;
            }
            element = element->Parent;
        }
        setChildrenClosestRobotLink(ClosestRobotLink);
    }

    int Id;
    int ControlId;
    bool IsControlled;
    std::shared_ptr<KinematicElement> Parent;
    std::vector<std::shared_ptr<KinematicElement>> Children;
    std::shared_ptr<KinematicElement> ClosestRobotLink;
    KDL::Segment Segment;
    KDL::Frame Frame;
    std::vector<double> JointLimits;
    shapes::ShapeConstPtr Shape;
    bool isRobotLink;
private:
    void setChildrenClosestRobotLink(std::shared_ptr<KinematicElement> element)
    {
        ClosestRobotLink = element;
        for(auto& child : Children)
        {
            child->setChildrenClosestRobotLink(element);
        }
    }
};

#endif // KINEMATICELEMENT_H
