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

#include <eigen_conversions/eigen_kdl.h>
#include <algorithm>

#include <exotica_core/tools/conversions.h>
#include <exotica_core/tools/printable.h>

namespace Eigen
{
Eigen::VectorXd VectorTransform(double px, double py, double pz, double qx, double qy, double qz, double qw)
{
    Eigen::VectorXd ret((Eigen::VectorXd(7) << px, py, pz, qx, qy, qz, qw).finished());
    return ret;
}

Eigen::VectorXd IdentityTransform()
{
    return VectorTransform();
}
}  // namespace Eigen

namespace exotica
{
KDL::Frame GetFrame(Eigen::VectorXdRefConst val)
{
    switch (val.rows())
    {
        case 7:
        {
            double norm = val.tail(4).norm();
            if (norm <= 0.0) ThrowPretty("Invalid quaternion!");
            return KDL::Frame(KDL::Rotation::Quaternion(val(3) / norm, val(4) / norm, val(5) / norm, val(6) / norm), KDL::Vector(val(0), val(1), val(2)));
        }
        case 6:
            return KDL::Frame(KDL::Rotation::RPY(val(3), val(4), val(5)), KDL::Vector(val(0), val(1), val(2)));
        case 3:
            return KDL::Frame(KDL::Vector(val(0), val(1), val(2)));
        default:
            ThrowPretty("Eigen vector has incorrect length! (" + std::to_string(val.rows()) + ")");
    }
}

KDL::Frame GetFrameFromMatrix(Eigen::MatrixXdRefConst val)
{
    switch (val.cols())
    {
        case 1:
            return GetFrame(val);
        case 4:
            switch (val.rows())
            {
                case 3:
                case 4:
                    return KDL::Frame(KDL::Rotation(val(0, 0), val(0, 1), val(0, 2), val(1, 0), val(1, 1), val(1, 2), val(2, 0), val(2, 1), val(2, 2)), KDL::Vector(val(0, 3), val(1, 3), val(2, 3)));
                default:
                    ThrowPretty("Eigen matrix has incorrect number of rows! (" + std::to_string(val.rows()) + ")");
            }
        default:
            ThrowPretty("Eigen matrix has incorrect number of columns! (" + std::to_string(val.cols()) + ")");
    }
}

Eigen::MatrixXd GetFrame(const KDL::Frame& val)
{
    Eigen::Isometry3d ret;
    tf::transformKDLToEigen(val, ret);
    return ret.matrix();
}

Eigen::VectorXd GetFrameAsVector(const KDL::Frame& val, RotationType type)
{
    const int rotation_length = GetRotationTypeLength(type);
    Eigen::VectorXd ret(3 + rotation_length);
    ret(0) = val.p[0];
    ret(1) = val.p[1];
    ret(2) = val.p[2];
    ret.segment(3, rotation_length) = SetRotation(val.M, type);
    return ret;
}

Eigen::VectorXd GetRotationAsVector(const KDL::Frame& val, RotationType type)
{
    return GetFrameAsVector(val, type).tail(GetRotationTypeLength(type));
}

KDL::Rotation GetRotation(Eigen::VectorXdRefConst data, RotationType type)
{
    switch (type)
    {
        case RotationType::QUATERNION:
            if (data.sum() == 0.0) ThrowPretty("Invalid quaternion transform!");
            return KDL::Rotation::Quaternion(data(0), data(1), data(2), data(3));
        case RotationType::RPY:
            return KDL::Rotation::RPY(data(0), data(1), data(2));
        case RotationType::ZYX:
            return KDL::Rotation::EulerZYX(data(0), data(1), data(2));
        case RotationType::ZYZ:
            return KDL::Rotation::EulerZYZ(data(0), data(1), data(2));
        case RotationType::ANGLE_AXIS:
        {
            KDL::Vector axis(data(0), data(1), data(2));
            double angle = axis.Norm();
            if (fabs(angle) > 1e-10)
            {
                return KDL::Rotation::Rot(axis, angle);
            }
            else
            {
                return KDL::Rotation();
            }
        }
        case RotationType::MATRIX:
            if (data.sum() == 0.0) ThrowPretty("Invalid matrix transform!");
            return KDL::Rotation(data(0), data(1), data(2),
                                 data(3), data(4), data(5),
                                 data(6), data(7), data(8));
        default:
            ThrowPretty("Unknown rotation representation type!");
    }
}

Eigen::VectorXd SetRotation(const KDL::Rotation& data, RotationType type)
{
    Eigen::VectorXd ret;
    switch (type)
    {
        case RotationType::QUATERNION:
            ret = Eigen::Quaterniond(Eigen::Map<const Eigen::Matrix3d>(data.data).transpose()).coeffs();
            return ret;
        case RotationType::RPY:
            ret.resize(3);
            data.GetRPY(ret(0), ret(1), ret(2));
            return ret;
        case RotationType::ZYX:
            ret.resize(3);
            data.GetEulerZYX(ret(0), ret(1), ret(2));
            return ret;
        case RotationType::ZYZ:
            ret.resize(3);
            data.GetEulerZYZ(ret(0), ret(1), ret(2));
            return ret;
        case RotationType::ANGLE_AXIS:
            ret = Eigen::Map<const Eigen::Vector3d>(data.GetRot().data);
            return ret;
        case RotationType::MATRIX:
            ret = Eigen::Map<const Eigen::VectorXd>(data.data, 9);
            return ret;
        default:
            ThrowPretty("Unknown rotation representation type!");
    }
}
}  // namespace exotica
