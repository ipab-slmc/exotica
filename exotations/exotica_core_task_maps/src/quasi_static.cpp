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

#include <cmath>

#include <exotica_core/server.h>
#include <exotica_core_task_maps/convex_hull.h>
#include <exotica_core_task_maps/quasi_static.h>

REGISTER_TASKMAP_TYPE("QuasiStatic", exotica::QuasiStatic);

namespace exotica
{
constexpr double eps = 1e-8;

QuasiStatic::QuasiStatic() = default;
QuasiStatic::~QuasiStatic() = default;

///
/// \brief cross 2D cross product (z coordinate of a 3D cross product of 2 vectors on a xy plane).
/// \param a 2D Vector.
/// \param b 2D Vector.
/// \return 2D cross product.
///
double cross(Eigen::VectorXdRefConst a, Eigen::VectorXdRefConst b)
{
    return a(0) * b(1) - a(1) * b(0);
}

///
/// \brief potential Calculates electrostatic potential at pont P induced by a uniformly charged line AB.
/// \param phi Potential.
/// \param A 1st point on the line.
/// \param B 2nd point on the line.
/// \param P Query point.
///
void potential(double& phi, Eigen::VectorXdRefConst A, Eigen::VectorXdRefConst B, Eigen::VectorXdRefConst P)
{
    double C = A.dot(B) - A.dot(P) + B.dot(P) - B.dot(B);
    double D = -A.dot(B) - A.dot(P) + B.dot(P) + A.dot(A);
    double E = cross(A, B) - cross(A, P) + cross(B, P);
    if (fabs(E) <= eps)
    {
        phi = 0.0;
    }
    else
    {
        phi = (atan(C / E) - atan(D / E)) / E;
    }
}

///
/// \brief potential Calculates electrostatic potential at pont P induced by a uniformly charged line AB.
/// \param phi Potential.
/// \param jacobian Gradient of the potential.
/// \param A 1st point on the line.
/// \param B 2nd point on the line.
/// \param P Query point.
/// \param A_ Derivative of 1st point on the line.
/// \param B_ Derivative of 2nd point on the line.
/// \param P_ Derivative of query point.
///
void potential(double& phi, Eigen::VectorXdRef jacobian, Eigen::VectorXdRefConst A, Eigen::VectorXdRefConst B, Eigen::VectorXdRefConst P, Eigen::MatrixXdRefConst A_, Eigen::MatrixXdRefConst B_, Eigen::MatrixXdRefConst P_)
{
    double C = A.dot(B) - A.dot(P) + B.dot(P) - B.dot(B);
    double D = -A.dot(B) - A.dot(P) + B.dot(P) + A.dot(A);
    double E = cross(A, B) - cross(A, P) + cross(B, P);
    if (fabs(E) < eps)
    {
        phi = 0.0;
        jacobian.setZero();
    }
    else
    {
        phi = (atan(C / E) - atan(D / E)) / E;
        for (int i = 0; i < jacobian.rows(); ++i)
        {
            double C_ = A_.col(i).dot(B) + A.dot(B_.col(i)) - A_.col(i).dot(P) - A.dot(P_.col(i)) + B_.col(i).dot(P) + B.dot(P_.col(i)) - 2 * B_.col(i).dot(B);
            double D_ = -A_.col(i).dot(B) - A.dot(B_.col(i)) - A_.col(i).dot(P) - A.dot(P_.col(i)) + B_.col(i).dot(P) + B.dot(P_.col(i)) + 2 * A_.col(i).dot(A);
            double E_ = cross(A_.col(i), B) + cross(A, B_.col(i)) - cross(A_.col(i), P) - cross(A, P_.col(i)) + cross(B_.col(i), P) + cross(B, P_.col(i));
            jacobian(i) = ((C_ / E - E_ * C / E / E) / (C * C / E / E + 1) - (D_ / E - E_ * D / E / E) / (D * D / E / E + 1)) / E - E_ / E * phi;
        }
    }
}

///
/// \brief winding Calculates the winding number around pont P w.r.t. thw line AB.
/// \param phi Winding number.
/// \param A 1st point on the line.
/// \param B 2nd point on the line.
/// \param P Query point.
///
void winding(double& phi, Eigen::VectorXdRefConst A, Eigen::VectorXdRefConst B, Eigen::VectorXdRefConst P)
{
    double C = cross(A - P, B - P);
    double D = (A - P).dot(B - P);
    phi = atan2(C, D) / 2.0 / M_PI;
}

///
/// \brief winding Calculates the winding number around pont P w.r.t. thw line AB.
/// \param phi Winding number.
/// \param jacobian Gradient of the Winding number.
/// \param A 1st point on the line.
/// \param B 2nd point on the line.
/// \param P Query point.
/// \param A_ Derivative of 1st point on the line.
/// \param B_ Derivative of 2nd point on the line.
/// \param P_ Derivative of query point.
///
void winding(double& phi, Eigen::VectorXdRef jacobian, Eigen::VectorXdRefConst A, Eigen::VectorXdRefConst B, Eigen::VectorXdRefConst P, Eigen::MatrixXdRefConst A_, Eigen::MatrixXdRefConst B_, Eigen::MatrixXdRefConst P_)
{
    double C = cross(A - P, B - P);
    double D = (A - P).dot(B - P);
    phi = atan2(C, D) / 2.0 / M_PI;
    for (int i = 0; i < jacobian.rows(); ++i)
    {
        double C_ = cross(A_.col(i) - P_.col(i), B - P) + cross(A - P, B_.col(i) - P_.col(i));
        double D_ = (A_.col(i) - P_.col(i)).dot(B - P) + (A - P).dot(B_.col(i) - P_.col(i));
        jacobian(i) = (C_ * D - C * D_) / (C * C + D * D) / 2.0 / M_PI;
    }
}

void QuasiStatic::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != 1) ThrowNamed("Wrong size of phi!");
    phi(0) = 0.0;
    KDL::Vector kdl_com;
    double M = 0.0;
    for (std::weak_ptr<KinematicElement> welement : scene_->GetKinematicTree().GetTree())
    {
        std::shared_ptr<KinematicElement> element = welement.lock();
        if (element->is_robot_link || element->closest_robot_link.lock())  // Only for robot links and attached objects
        {
            double mass = element->segment.getInertia().getMass();
            if (mass > 0)
            {
                KDL::Frame cog = KDL::Frame(element->segment.getInertia().getCOG());
                KDL::Frame com_local = scene_->GetKinematicTree().FK(element, cog, nullptr, KDL::Frame());
                kdl_com += com_local.p * mass;
                M += mass;
            }
        }
    }
    if (M == 0.0) return;
    kdl_com = kdl_com / M;
    Eigen::VectorXd com(2);
    com(0) = kdl_com[0];
    com(1) = kdl_com[1];

    Eigen::MatrixXd supports(kinematics[0].Phi.rows(), 2);
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        supports(i, 0) = kinematics[0].Phi(i).p[0];
        supports(i, 1) = kinematics[0].Phi(i).p[1];
    }

    std::list<int> hull = ConvexHull2D(supports);

    double n = hull.size();
    double wnd = 0.0;
    double pot = 0.0;
    double tmp;
    for (std::list<int>::iterator it = hull.begin(); it != hull.end();)
    {
        int a = *it;
        int b = ++it == hull.end() ? *hull.begin() : *(it);
        potential(tmp, supports.row(a).transpose(), supports.row(b).transpose(), com);
        pot += tmp;
        winding(tmp, supports.row(a).transpose(), supports.row(b).transpose(), com);
        wnd += tmp;
    }
    wnd = fabs(wnd);

    if (pot < eps)
    {
        if (wnd < 0.5)
        {
            phi(0) = -sqrt(-n / pot);
        }
        else
        {
            if (!parameters_.PositiveOnly)
            {
                phi(0) = sqrt(-n / pot);
            }
        }
    }

    if (debug_)
    {
        debug_msg_.markers[0].pose.position.x = com(0);
        debug_msg_.markers[0].pose.position.y = com(1);
        debug_msg_.markers[0].pose.position.z = 0.0;

        debug_msg_.markers[1].points.resize(hull.size() + 1);
        int ii = 0;
        for (int i : hull)
        {
            debug_msg_.markers[1].points[ii].x = supports(i, 0);
            debug_msg_.markers[1].points[ii].y = supports(i, 1);
            debug_msg_.markers[1].points[ii++].z = 0.0;
        }
        debug_msg_.markers[1].points[hull.size()] = debug_msg_.markers[1].points[0];

        debug_pub_.publish(debug_msg_);
    }
}

void QuasiStatic::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != 1) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != 1 || jacobian.cols() != x.rows()) ThrowNamed("Wrong size of jacobian! " << x.rows());
    phi(0) = 0.0;
    jacobian.setZero();
    Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(2, jacobian.cols());
    KDL::Vector kdl_com;
    double M = 0.0;
    for (std::weak_ptr<KinematicElement> welement : scene_->GetKinematicTree().GetTree())
    {
        std::shared_ptr<KinematicElement> element = welement.lock();
        if (element->is_robot_link || element->closest_robot_link.lock())  // Only for robot links and attached objects
        {
            double mass = element->segment.getInertia().getMass();
            if (mass > 0)
            {
                KDL::Frame cog = KDL::Frame(element->segment.getInertia().getCOG());
                KDL::Frame com_local = scene_->GetKinematicTree().FK(element, cog, nullptr, KDL::Frame());
                Eigen::MatrixXd jacobian_com_local = scene_->GetKinematicTree().Jacobian(element, cog, nullptr, KDL::Frame());
                kdl_com += com_local.p * mass;
                jacobian_com += jacobian_com_local.topRows(2) * mass;
                M += mass;
            }
        }
    }
    if (M == 0.0) return;
    kdl_com = kdl_com / M;
    Eigen::VectorXd com(2);
    com(0) = kdl_com[0];
    com(1) = kdl_com[1];
    jacobian_com = jacobian_com / M;

    Eigen::MatrixXd supports(kinematics[0].Phi.rows(), 2);
    Eigen::MatrixXd supportsJ(kinematics[0].Phi.rows() * 2, x.rows());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        supports(i, 0) = kinematics[0].Phi(i).p[0];
        supports(i, 1) = kinematics[0].Phi(i).p[1];
        supportsJ.middleRows(i * 2, 2) = kinematics[0].jacobian(i).data.topRows(2);
    }

    std::list<int> hull = ConvexHull2D(supports);

    double n = hull.size();
    double wnd = 0.0;
    double pot = 0.0;
    Eigen::VectorXd potJ = jacobian.row(0);
    double tmp;
    Eigen::VectorXd tmpJ = jacobian.row(0);
    for (std::list<int>::iterator it = hull.begin(); it != hull.end(); ++it)
    {
        int a = *it;
        int b = (std::next(it) == hull.end()) ? *hull.begin() : *(std::next(it));
        potential(tmp, tmpJ, supports.row(a).transpose(), supports.row(b).transpose(), com, supportsJ.middleRows(a * 2, 2), supportsJ.middleRows(b * 2, 2), jacobian_com);
        pot += tmp;
        potJ += tmpJ;
        winding(tmp, supports.row(a).transpose(), supports.row(b).transpose(), com);
        wnd += tmp;
    }
    wnd = fabs(wnd);

    if (fabs(pot) > eps)
    {
        if (wnd < 0.5)
        {
            phi(0) = sqrt(-n / pot);
            jacobian.row(0) = potJ * (n / (2 * pot * pot * phi(0)));
        }
        else
        {
            if (!parameters_.PositiveOnly)
            {
                phi(0) = -sqrt(-n / pot);
                jacobian.row(0) = potJ * (n / (2 * pot * pot * phi(0)));
            }
        }
    }

    if (debug_)
    {
        debug_msg_.markers[0].pose.position.x = com(0);
        debug_msg_.markers[0].pose.position.y = com(1);
        debug_msg_.markers[0].pose.position.z = 0.0;

        debug_msg_.markers[1].points.resize(hull.size() + 1);
        int ii = 0;
        for (int i : hull)
        {
            debug_msg_.markers[1].points[ii].x = supports(i, 0);
            debug_msg_.markers[1].points[ii].y = supports(i, 1);
            debug_msg_.markers[1].points[ii].z = 0.0;
            ++ii;
        }
        debug_msg_.markers[1].points[ii] = debug_msg_.markers[1].points[0];

        debug_pub_.publish(debug_msg_);
    }
}

int QuasiStatic::TaskSpaceDim()
{
    return 1;
}

void QuasiStatic::Initialize()
{
    {
        visualization_msgs::Marker mrk;
        mrk.action = visualization_msgs::Marker::ADD;
        mrk.header.frame_id = "exotica/" + scene_->GetRootFrameName();
        mrk.id = 1;
        mrk.type = visualization_msgs::Marker::SPHERE;
        mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.05;
        mrk.color.r = 0;
        mrk.color.g = 1;
        mrk.color.b = 0;
        mrk.color.a = 1;
        debug_msg_.markers.push_back(mrk);
    }
    {
        visualization_msgs::Marker mrk;
        mrk.action = visualization_msgs::Marker::ADD;
        mrk.header.frame_id = "exotica/" + scene_->GetRootFrameName();
        mrk.id = 2;
        mrk.type = visualization_msgs::Marker::LINE_STRIP;
        mrk.color.r = 0;
        mrk.color.g = 0;
        mrk.color.b = 1;
        mrk.color.a = 1;
        mrk.scale.x = 0.02;
        debug_msg_.markers.push_back(mrk);
    }
    if (debug_)
    {
        debug_pub_ = Server::Advertise<visualization_msgs::MarkerArray>(ns_ + "/exotica/QuasiStatic", 1, true);
    }
}

void QuasiStatic::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}
}
