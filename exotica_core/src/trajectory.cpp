#include <exotica_core/trajectory.h>
#include <iostream>
#include <string>

namespace exotica
{
Trajectory::Trajectory() : radius_(1.0), trajectory_(nullptr)
{
}

Trajectory::Trajectory(const std::string& data)
{
    std::istringstream ss(data);
    ss >> radius_;
    int n, m;
    ss >> n;
    ss >> m;
    data_.resize(n, m);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            double val;
            ss >> val;
            data_(i, j) = val;
        }
    }
    constructFromData(data_, radius_);
}

Trajectory::Trajectory(Eigen::MatrixXdRefConst data, double radius)
{
    constructFromData(data, radius);
}

KDL::Frame Trajectory::getPosition(double t)
{
    return trajectory_->Pos(t);
}

KDL::Twist Trajectory::getVelocity(double t)
{
    return trajectory_->Vel(t);
}

KDL::Twist Trajectory::getAcceleration(double t)
{
    return trajectory_->Acc(t);
}

double Trajectory::getDuration()
{
    return trajectory_->Duration();
}

Eigen::MatrixXd Trajectory::getData()
{
    return data_;
}

double Trajectory::getRadius()
{
    return radius_;
}

std::string Trajectory::toString()
{
    std::ostringstream ss;
    ss << radius_ << "\n";
    ss << data_.rows() << " " << data_.cols() << "\n";
    ss << data_;
    return ss.str();
}

void Trajectory::constructFromData(Eigen::MatrixXdRefConst data, double radius)
{
    if (!(data.cols() == 4 || data.cols() == 7 || data.cols() == 8) || data.rows() < 2) throw_pretty("Invalid trajectory data size!");
    trajectory_.reset(new KDL::Trajectory_Composite());
    for (int i = 0; i < data.rows() - 1; i++)
    {
        KDL::Frame f1 = getFrame(data.row(i).tail(data.cols() - 1).transpose());
        KDL::Frame f2 = getFrame(data.row(i + 1).tail(data.cols() - 1).transpose());
        double dt = data(i + 1, 0) - data(i, 0);
        if (dt <= 0) throw_pretty("Time indices must be monotonically increasing! " << i << " (" << dt << ")");
        if (KDL::Equal(f1, f2, 1e-6))
        {
            trajectory_->Add(new KDL::Trajectory_Stationary(dt, f1));
        }
        else
        {
            KDL::RotationalInterpolation_SingleAxis* rot = new KDL::RotationalInterpolation_SingleAxis();
            KDL::Path_Line* path = new KDL::Path_Line(f1, f2, rot, radius);
            KDL::VelocityProfile_Spline* vel = new KDL::VelocityProfile_Spline();
            KDL::Trajectory_Segment* seg = new KDL::Trajectory_Segment(path, vel, dt);
            trajectory_->Add(seg);
        }
    }
    data_ = data;
    radius_ = radius;
}
}
