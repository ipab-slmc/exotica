#ifndef EXOTICA_TRAJECTORY_H
#define EXOTICA_TRAJECTORY_H

#include <memory>

#include <Eigen/Dense>

#include <kdl/path.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/velocityprofile.hpp>
#include <kdl/velocityprofile_spline.hpp>

#include <exotica/Tools.h>

namespace exotica
{
class Trajectory
{
public:
    Trajectory();
    Trajectory(const std::string& data);
    Trajectory(Eigen::MatrixXdRefConst data, double radius = 1.0);
    ~Trajectory() {}
    KDL::Frame getPosition(double t);
    KDL::Twist getVelocity(double t);
    KDL::Twist getAcceleration(double t);
    double getDuration();
    Eigen::MatrixXd getData();
    double getRadius();
    std::string toString();

protected:
    void constructFromData(Eigen::MatrixXdRefConst data, double radius);
    double radius_;
    Eigen::MatrixXd data_;
    std::shared_ptr<KDL::Trajectory_Composite> trajectory_;
};
}

#endif
