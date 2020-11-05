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

#ifndef EXOTICA_CORE_TOOLS_H_
#define EXOTICA_CORE_TOOLS_H_

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <string>

#include <exotica_core/tools/conversions.h>
#include <exotica_core/tools/exception.h>
#include <exotica_core/tools/printable.h>
#include <exotica_core/tools/timer.h>
#include <exotica_core/tools/uncopyable.h>
#include <exotica_core/version.h>

#include <geometric_shapes/shapes.h>
#include <octomap/OcTree.h>
#include <std_msgs/ColorRGBA.h>

/**
 * \brief A double-wrapper MACRO functionality for generating unique object names: The actual functionality is provided by EX_UNIQ (for 'exotica unique')
 */
#define EX_CONC(x, y) x##y
#define EX_UNIQ(x, y) EX_CONC(x, y)

namespace exotica
{
/**
   * @brief RandomColor Generates random opaque color.
   * @return Random color
   */
std_msgs::ColorRGBA RandomColor();
inline std_msgs::ColorRGBA GetColor(double r, double g, double b, double a = 1.0)
{
    std_msgs::ColorRGBA ret;
    ret.r = static_cast<float>(r);
    ret.g = static_cast<float>(g);
    ret.b = static_cast<float>(b);
    ret.a = static_cast<float>(a);
    return ret;
}

inline std_msgs::ColorRGBA GetColor(const Eigen::Vector4d& rgba)
{
    std_msgs::ColorRGBA ret;
    ret.r = static_cast<float>(rgba(0));
    ret.g = static_cast<float>(rgba(1));
    ret.b = static_cast<float>(rgba(2));
    ret.a = static_cast<float>(rgba(3));
    return ret;
}

/**
   * @brief LoadOBJ Loads mesh data from an OBJ file
   * @param file_name File name
   * @param tri Returned vertex indices of triangles
   * @param vert Vertex positions
   */
void LoadOBJ(const std::string& data, Eigen::VectorXi& tri,
             Eigen::VectorXd& vert);

std::shared_ptr<octomap::OcTree> LoadOctree(const std::string& file_path);

std::shared_ptr<shapes::Shape> LoadOctreeAsShape(const std::string& file_path);

void SaveMatrix(std::string file_name,
                const Eigen::Ref<const Eigen::MatrixXd> mat);

template <typename T>
std::vector<std::string> GetKeys(std::map<std::string, T> map)
{
    std::vector<std::string> ret;
    for (auto& it : map) ret.push_back(it.first);
    return ret;
}

std::string GetTypeName(const std::type_info& type);

std::string ParsePath(const std::string& path);

std::string LoadFile(const std::string& path);

bool PathExists(const std::string& path);

/// \brief Argument position.
///        Used as parameter to refer to an argument.
enum ArgumentPosition
{
    ARG0 = 0,
    ARG1 = 1,
    ARG2 = 2,
    ARG3 = 3,
    ARG4 = 4
};
}

namespace
{
template <class SharedPointer>
struct Holder
{
    SharedPointer p;

    Holder(const SharedPointer& p) : p(p) {}
    Holder(const Holder& other) : p(other.p) {}
    Holder(Holder&& other) : p(std::move(other.p)) {}
    void operator()(...) { p.reset(); }
};
}

template <class T>
std::shared_ptr<T> ToStdPtr(const boost::shared_ptr<T>& p)
{
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
}

template <class T>
std::shared_ptr<T> ToStdPtr(const std::shared_ptr<T>& p)
{
    return p;
}

#endif  // EXOTICA_CORE_TOOLS_H_
