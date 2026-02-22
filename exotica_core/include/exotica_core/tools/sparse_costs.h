//
// Copyright (c) 2019-2020, University of Edinburgh, University of Oxford
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

#ifndef EXOTICA_CORE_TOOLS_SPARSE_COSTS_H_
#define EXOTICA_CORE_TOOLS_SPARSE_COSTS_H_

#include <exotica_core/tools/exception.h>
#include <Eigen/Dense>
#include <vector>

namespace exotica
{
inline double huber_cost(double x, double beta)
{
    if (std::abs(x) < beta)
        return 0.5 * x * x;
    else
        return beta * (std::abs(x) - 0.5 * beta);
}

inline double huber_jacobian(double x, double beta)
{
    if (std::abs(x) < beta)
        return x;
    if (x < 0)
        return -beta;
    return beta;
}

inline double huber_hessian(double x, double beta)
{
    if (std::abs(x) < beta)
        return 1;
    return 0;
}

inline double smooth_l1_cost(double x, double beta)
{
    if (std::abs(x) < beta)
        return 0.5 * x * x / beta;
    return std::abs(x) - 0.5 * beta;
}

inline double smooth_l1_jacobian(double x, double beta)
{
    if (std::abs(x) < beta)
        return x / beta;
    if (x < -beta)
        return -1.0;
    return 1.0;
}

inline double smooth_l1_hessian(double x, double beta)
{
    if (std::abs(x) < beta)
        return 1.0 / beta;
    return 0.0;
}

inline double pseudo_huber_cost(double x, double beta)
{
    return beta * beta * (std::sqrt(1 + std::pow(x / beta, 2)) - 1.0);
}

inline double pseudo_huber_jacobian(double x, double beta)
{
    return x / (std::sqrt(1.0 + x * x / (beta * beta)));
}

inline double pseudo_huber_hessian(double x, double beta)
{
    return std::pow(beta, 4) * std::sqrt(1.0 + x * x / (beta * beta)) / (std::pow(beta * beta + x * x, 2));
}

}  // namespace exotica

#endif  // EXOTICA_CORE_TOOLS_SPARSE_COSTS_H_
