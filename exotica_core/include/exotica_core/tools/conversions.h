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

#ifndef EXOTICA_CORE_CONVERSIONS_H_
#define EXOTICA_CORE_CONVERSIONS_H_

#include <map>
#include <memory>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Dense>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#include <unsupported/Eigen/CXX11/Tensor>
#pragma GCC diagnostic pop

#include <exotica_core/tools/exception.h>
#include <exotica_core/tools/printable.h>

namespace Eigen
{
/// \brief Convenience wrapper for storing references to sub-matrices/vectors
typedef Ref<VectorXd> VectorXdRef;
typedef const Ref<const VectorXd>& VectorXdRefConst;
typedef Ref<MatrixXd> MatrixXdRef;
typedef const Ref<const MatrixXd>& MatrixXdRefConst;

Eigen::VectorXd VectorTransform(double px = 0.0, double py = 0.0, double pz = 0.0, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
Eigen::VectorXd IdentityTransform();

template <typename T>
using MatrixType = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
template <typename Scalar, int rank, typename sizeType>
inline MatrixType<Scalar> TensorToMatrix(const Eigen::Tensor<Scalar, rank>& tensor, const sizeType rows, const sizeType cols)
{
    return Eigen::Map<const MatrixType<Scalar>>(tensor.data(), rows, cols);
}

template <typename Scalar, typename... Dims>
inline Eigen::Tensor<Scalar, sizeof...(Dims)> MatrixToTensor(const MatrixType<Scalar>& matrix, Dims... dims)
{
    constexpr int rank = sizeof...(Dims);
    return Eigen::TensorMap<Eigen::Tensor<const Scalar, rank>>(matrix.data(), {dims...});
}

}  // namespace Eigen

namespace exotica
{
enum class RotationType
{
    QUATERNION,
    RPY,
    ZYX,
    ZYZ,
    ANGLE_AXIS,
    MATRIX
};

KDL::Rotation GetRotation(Eigen::VectorXdRefConst data, RotationType type);

Eigen::VectorXd SetRotation(const KDL::Rotation& data, RotationType type);

inline int GetRotationTypeLength(const RotationType& type)
{
    static constexpr int types[] = {4, 3, 3, 3, 3, 9};
    return types[static_cast<int>(type)];
}

inline RotationType GetRotationTypeFromString(const std::string& rotation_type)
{
    if (rotation_type == "Quaternion")
    {
        return RotationType::QUATERNION;
    }
    else if (rotation_type == "RPY")
    {
        return RotationType::RPY;
    }
    else if (rotation_type == "ZYX")
    {
        return RotationType::ZYX;
    }
    else if (rotation_type == "ZYZ")
    {
        return RotationType::ZYZ;
    }
    else if (rotation_type == "AngleAxis")
    {
        return RotationType::ANGLE_AXIS;
    }
    else if (rotation_type == "Matrix")
    {
        return RotationType::MATRIX;
    }
    else
    {
        ThrowPretty("Unsupported rotation type '" << rotation_type << "'");
    }
}

KDL::Frame GetFrame(Eigen::VectorXdRefConst val);

KDL::Frame GetFrameFromMatrix(Eigen::MatrixXdRefConst val);

Eigen::MatrixXd GetFrame(const KDL::Frame& val);

Eigen::VectorXd GetFrameAsVector(const KDL::Frame& val, RotationType type = RotationType::RPY);

Eigen::VectorXd GetRotationAsVector(const KDL::Frame& val, RotationType type);

inline void NormalizeQuaternionInConfigurationVector(Eigen::Ref<Eigen::VectorXd> q)
{
    q.segment<4>(3) = Eigen::Quaterniond(q.segment<4>(3)).normalized().coeffs();
}

inline void SetDefaultQuaternionInConfigurationVector(Eigen::Ref<Eigen::VectorXd> q)
{
    q.segment<4>(3) = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).normalized().coeffs();
}

typedef Eigen::Array<KDL::Frame, Eigen::Dynamic, 1> ArrayFrame;
typedef Eigen::Array<KDL::Twist, Eigen::Dynamic, 1> ArrayTwist;
typedef Eigen::Array<KDL::Jacobian, Eigen::Dynamic, 1> ArrayJacobian;
typedef Eigen::Array<Eigen::MatrixXd, Eigen::Dynamic, 1> Hessian;
typedef Eigen::Array<Eigen::Array<Eigen::MatrixXd, Eigen::Dynamic, 1>, Eigen::Dynamic, 1> ArrayHessian;
typedef Eigen::Ref<Eigen::Array<KDL::Frame, Eigen::Dynamic, 1>> ArrayFrameRef;
typedef Eigen::Ref<Eigen::Array<KDL::Twist, Eigen::Dynamic, 1>> ArrayTwistRef;
typedef Eigen::Ref<Eigen::Array<KDL::Jacobian, Eigen::Dynamic, 1>> ArrayJacobianRef;
typedef Eigen::Ref<Eigen::Array<Eigen::MatrixXd, Eigen::Dynamic, 1>> HessianRef;
typedef Eigen::Ref<Eigen::Array<Eigen::Array<Eigen::MatrixXd, Eigen::Dynamic, 1>, Eigen::Dynamic, 1>> ArrayHessianRef;
typedef const Eigen::Ref<Eigen::Array<KDL::Frame, Eigen::Dynamic, 1>>& ArrayFrameRefConst;
typedef const Eigen::Ref<Eigen::Array<KDL::Twist, Eigen::Dynamic, 1>>& ArrayTwistRefConst;
typedef const Eigen::Ref<Eigen::Array<KDL::Jacobian, Eigen::Dynamic, 1>>& ArrayJacobianRefConst;
typedef const Eigen::Ref<Eigen::Array<Eigen::MatrixXd, Eigen::Dynamic, 1>> HessianRefConst;
typedef const Eigen::Ref<Eigen::Array<Eigen::Array<Eigen::MatrixXd, Eigen::Dynamic, 1>, Eigen::Dynamic, 1>> ArrayHessianRefConst;

inline bool IsContainerType(std::string type)
{
    return type == "exotica::Initializer";
}

inline bool IsVectorType(std::string type)
{
    return type.substr(0, 11) == "std::vector";
}

inline bool IsVectorContainerType(std::string type)
{
    return type == "std::vector<exotica::Initializer>";
}

template <class Key, class Val>
[[deprecated("Replaced by GetKeysFromMap and GetValuesFromMap")]] std::vector<Val> MapToVec(const std::map<Key, Val>& map)
{
    std::vector<Val> ret;
    for (auto& it : map)
    {
        ret.push_back(it.second);
    }
    return ret;
}

template <class Key, class Val>
std::vector<Key> GetKeysFromMap(const std::map<Key, Val>& map)
{
    std::vector<Key> ret;
    for (auto& it : map)
    {
        ret.push_back(it.first);
    }
    return ret;
}

template <class Key, class Val>
std::vector<Val> GetValuesFromMap(const std::map<Key, Val>& map)
{
    std::vector<Val> ret;
    for (auto& it : map)
    {
        ret.push_back(it.second);
    }
    return ret;
}

template <class Key, class Val>
void AppendMap(std::map<Key, Val>& orig, const std::map<Key, Val>& extra)
{
    orig.insert(extra.begin(), extra.end());
}

template <class Val>
void AppendVector(std::vector<Val>& orig, const std::vector<Val>& extra)
{
    orig.insert(orig.end(), extra.begin(), extra.end());
}

inline std::string Trim(const std::string& s)
{
    auto wsfront = std::find_if_not(s.begin(), s.end(), [](int c) { return std::isspace(c); });
    return std::string(wsfront, std::find_if_not(s.rbegin(), std::string::const_reverse_iterator(wsfront), [](int c) { return std::isspace(c); }).base());
}

template <typename T>
T ToNumber(const std::string& val)
{
    throw std::runtime_error("conversion not implemented!");
}

template <>
inline float ToNumber<float>(const std::string& val)
{
    return std::stof(val);
}

template <>
inline double ToNumber<double>(const std::string& val)
{
    return std::stod(val);
}

template <>
inline int ToNumber<int>(const std::string& val)
{
    return std::stoi(val);
}

template <typename T, const int S>  // Eigen::Vector<S><T>
inline Eigen::Matrix<T, S, 1> ParseVector(const std::string value)
{
    Eigen::Matrix<T, S, 1> ret;
    std::string temp_entry;
    int i = 0;

    std::istringstream text_parser(value);

    while (text_parser >> temp_entry)
    {
        ret.conservativeResize(++i);
        try
        {
            ret[i - 1] = ToNumber<T>(temp_entry);
        }
        catch (const std::invalid_argument& /* e */)
        {
            ret[i - 1] = std::numeric_limits<T>::quiet_NaN();
        }
    }
    if (i == 0) WARNING_NAMED("Parser", "Empty vector!")
    if (S != Eigen::Dynamic && S != i)
    {
        ThrowPretty("Wrong vector size! Requested: " + std::to_string(S) + ", Provided: " + std::to_string(i));
    }
    return ret;
}

inline bool ParseBool(const std::string value)
{
    bool ret;
    std::istringstream text_parser(value);
    text_parser >> ret;
    return ret;
}

inline double ParseDouble(const std::string value)
{
    double ret;
    std::istringstream text_parser(value);

    text_parser >> ret;
    if ((text_parser.fail() || text_parser.bad()))
    {
        ThrowPretty("Can't parse value!");
    }
    return ret;
}

inline int ParseInt(const std::string value)
{
    int ret;
    std::istringstream text_parser(value);

    text_parser >> ret;
    if ((text_parser.fail() || text_parser.bad()))
    {
        ThrowPretty("Can't parse value!");
    }
    return ret;
}

inline std::vector<std::string> ParseList(const std::string& value, char token = ',')
{
    std::stringstream ss(value);
    std::string item;
    std::vector<std::string> ret;
    while (std::getline(ss, item, token))
    {
        ret.push_back(Trim(item));
    }
    if (ret.size() == 0) WARNING_NAMED("Parser", "Empty vector!")
    return ret;
}

inline std::vector<int> ParseIntList(const std::string value)
{
    std::stringstream ss(value);
    std::string item;
    std::vector<int> ret;
    while (std::getline(ss, item, ' '))
    {
        int tmp;
        std::istringstream text_parser(item);
        text_parser >> tmp;
        if ((text_parser.fail() || text_parser.bad()))
        {
            ThrowPretty("Can't parse value!");
        }
        ret.push_back(tmp);
    }
    if (ret.size() == 0) WARNING_NAMED("Parser", "Empty vector!")
    return ret;
}

inline std::vector<bool> ParseBoolList(const std::string value)
{
    std::stringstream ss(value);
    std::string item;
    std::vector<bool> ret;
    while (std::getline(ss, item, ' '))
    {
        bool tmp;
        std::istringstream text_parser(item);
        text_parser >> tmp;
        if ((text_parser.fail() || text_parser.bad()))
        {
            ThrowPretty("Can't parse value!");
        }
        ret.push_back(tmp);
    }
    if (ret.empty()) WARNING_NAMED("Parser", "Empty vector!")
    return ret;
}
}

#endif  // EXOTICA_CORE_CONVERSIONS_H_
