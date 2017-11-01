/*
 *      Author: Michael Camilleri
 * 
 * Copyright (c) 2016, University Of Edinburgh 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of  nor the names of its contributors may be used to 
 *    endorse or promote products derived from this software without specific 
 *    prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#ifndef EXOTICA_TOOLS_H
#define EXOTICA_TOOLS_H

#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <string>

#include <exotica/Tools/Conversions.h>
#include <exotica/Tools/Exception.h>
#include <exotica/Tools/Printable.h>
#include <exotica/Tools/Timer.h>
#include <exotica/Tools/Uncopyable.h>
#include <exotica/Version.h>
#include <std_msgs/ColorRGBA.h>

/**
 * \brief A double-wrapper MACRO functionality for generating unique object names: The actual functionality is provided by EX_UNIQ (for 'exotica unique')
 */
#define EX_CONC(x, y) x##y
#define EX_UNIQ(x, y) EX_CONC(x, y)

namespace exotica
{
/**
   * @brief randomColor Generates random opaque color.
   * @return Random color
   */
std_msgs::ColorRGBA randomColor();
inline std_msgs::ColorRGBA getColor(double r, double g, double b, double a = 1.0)
{
    std_msgs::ColorRGBA ret;
    ret.r = r;
    ret.g = g;
    ret.b = b;
    ret.a = a;
    return ret;
}

/**
   * @brief loadOBJ Loads mesh data from an OBJ file
   * @param file_name File name
   * @param tri Returned vertex indices of triangles
   * @param vert Vertex positions
   */
void loadOBJ(const std::string& data, Eigen::VectorXi& tri,
             Eigen::VectorXd& vert);

void saveMatrix(std::string file_name,
                const Eigen::Ref<const Eigen::MatrixXd> mat);

void getText(std::string& txt, KDL::Frame& ret);

template <typename T>
std::vector<std::string> getKeys(std::map<std::string, T> map)
{
    std::vector<std::string> ret;
    for (auto& it : map) ret.push_back(it.first);
    return ret;
}

std::string getTypeName(const std::type_info& type);

std::string parsePath(const std::string& path);

std::string loadFile(const std::string& path);

bool pathExists(const std::string& path);
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
std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T>& p)
{
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
}

template <class T>
std::shared_ptr<T> to_std_ptr(const std::shared_ptr<T>& p)
{
    return p;
}

#endif
