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

#ifndef EXOTICA_CORE_PRINTABLE_H_
#define EXOTICA_CORE_PRINTABLE_H_

#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>

/// \brief A set of debugging tools: basically these provide easy ways of checking code execution through std::cout prints
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#ifdef EXOTICA_CORE_DEBUG_MODE
#define CHECK_EXECUTION std::cout << "\033[1;32m[EXOTica]:\033[0m Ok in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n";                        //!< With endline
#define INDICATE_FAILURE std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";  //!< With endline
#define WARNING(x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";                                                           //!< With endline
#define ERROR(x) std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" \
                           << x << "\033[0m\n";                                                                   // With endline
#define INFO(x) std::clog << "\033[1;32m[EXOTica]:\033[0m Info in " << __PRETTY_FUNCTION__ << ": " << x << "\n";  // With endline
#else
#define CHECK_EXECUTION                                                                                                                                                                         // No operation
#define INDICATE_FAILURE std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";  //!< With endline
#define WARNING(x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";                                                           //!< With endline
#define ERROR(x) std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" \
                           << x << "\033[0m\n";  // With endline
#define INFO(x)
#endif
#define HIGHLIGHT(x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[36m" << x << "\033[0m\n";
#define HIGHLIGHT_NAMED(name, x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name << "]\033[0m \033[36m" << x << "\033[0m\n";
#define WARNING_NAMED(name, x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name << "]\033[0m \033[33m" << x << "\033[0m\n";
#define INFO_NAMED(name, x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name << "]\033[0m " << x << "\n";
#define INFO_PLAIN(x) std::cout << x << "\n";

namespace exotica
{
class Printable
{
public:
    virtual void Print(std::ostream& os) const = 0;
};

std::ostream& operator<<(std::ostream& os, const Printable& s);

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& s)
{
    for (auto& p : s) os << p << "\n";
    return os;
}

template <typename I, typename T>
std::ostream& operator<<(std::ostream& os, const std::map<I, T>& s)
{
    for (auto& p : s) os << p.first << ": " << p.second << "\n";
    return os;
}

std::string ToString(const KDL::Frame& s);

std::string ToString(const Eigen::Isometry3d& s);

void PrintDimensions(const std::string& name, const Eigen::Ref<const Eigen::MatrixXd> m);
}  // namespace exotica

#endif  // EXOTICA_CORE_PRINTABLE_H_
