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

#include <exotica_core/tools/printable.h>

namespace exotica
{
std::ostream& operator<<(std::ostream& os, const Printable& s)
{
    s.Print(os);
    return os;
}

std::string ToString(const KDL::Frame& s)
{
    double x, y, z, w;
    s.M.GetQuaternion(x, y, z, w);
    return "([" + std::to_string(s.p.data[0]) + " " + std::to_string(s.p.data[1]) + " " + std::to_string(s.p.data[2]) + "] [" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(w) + "])";
}

std::string ToString(const Eigen::Isometry3d& s)
{
    Eigen::Quaterniond quat(s.linear());
    return "([" + std::to_string(s.translation().x()) + " " + std::to_string(s.translation().y()) + " " + std::to_string(s.translation().z()) + "] [" + std::to_string(quat.x()) + " " + std::to_string(quat.y()) + " " + std::to_string(quat.z()) + " " + std::to_string(quat.w()) + "])";
}

void PrintDimensions(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd> m)
{
    INFO_NAMED(name, m.rows() << "x" << m.cols());
}
}  // namespace exotica
