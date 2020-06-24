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
inline double huber_cost(double x, double delta)
{
    return delta * delta * (std::sqrt(1 + std::pow(x / delta, 2)) - 1.0);
}

inline double huber_jacobian(double x, double delta)
{
    return x / (std::sqrt(1.0 + x * x / (delta * delta)));
}

inline double huber_hessian(double x, double delta)
{
    return std::pow(delta, 4) * std::sqrt(1.0 + x * x / (delta * delta)) / (std::pow(delta * delta + x * x, 2));
}

inline double super_huber_cost(double x, double delta, double factor)
{
    return pow(delta, 2) * (pow(1 + pow(x, 2) / pow(delta, 2), factor) - 1);
}

inline double super_huber_jacobian(double x, double delta, double factor)
{
    return 2 * factor * x * pow(1 + pow(x, 2) / pow(delta, 2), factor) / (1 + pow(x, 2) / pow(delta, 2));
}

inline double super_huber_hessian(double x, double delta, double factor)
{
    return 2 * factor * pow(1 + pow(x, 2) / pow(delta, 2), factor) / (1 + pow(x, 2) / pow(delta, 2)) + 4 * pow(factor, 2) * pow(x, 2) * pow(1 + pow(x, 2) / pow(delta, 2), factor) / (pow(delta, 2) * pow(1 + pow(x, 2) / pow(delta, 2), 2)) - 4 * factor * pow(x, 2) * pow(1 + pow(x, 2) / pow(delta, 2), factor) / (pow(delta, 2) * pow(1 + pow(x, 2) / pow(delta, 2), 2));
}

inline double smooth_l1_cost(double x, double alpha)
{
    return 1.0 / alpha * (std::log(1.0 + std::exp(-alpha * x)) + std::log(1.0 + std::exp(alpha * x)));
}

inline double smooth_l1_jacobian(double x, double alpha)
{
    return 1.0 / (1 + std::exp(-alpha * x)) - 1.0 / (1 + std::exp(alpha * x));
}

inline double smooth_l1_hessian(double x, double alpha)
{
    return 2 * alpha * std::exp(alpha * x) / std::pow(1 + std::exp(alpha * x), 2);
}

inline double bimodal_huber_cost(double x, double delta, double mode1, double mode2)
{
    return huber_cost(x - mode1, delta) + huber_cost(x - mode2, delta) - huber_cost(x - (mode1 + mode2) / 2, delta) - huber_cost(mode1, delta);
}

inline double bimodal_huber_jacobian(double x, double delta, double mode1, double mode2)
{
    return huber_jacobian(x - mode1, delta) + huber_jacobian(x - mode2, delta) - huber_jacobian(x - (mode1 + mode2) / 2, delta);
}

inline double bimodal_huber_hessian(double x, double delta, double mode1, double mode2)
{
    return huber_hessian(x - mode1, delta) + huber_hessian(x - mode2, delta) - huber_hessian(x - (mode1 + mode2) / 2, delta);
}

inline double normalized_huber_cost(double x, double delta)
{
    return (
               1.0 * (delta * delta + 1.0) * (std::sqrt(std::pow(x / delta, 2) + 1.0) - 1.0)) /
           (std::sqrt(1 + 1 / (delta * delta)) * (delta + 1.0));
}

inline double normalized_huber_jacobian(double x, double delta)
{
    const double epsilon = 1e-7;
    x = x + epsilon;

    return (
               1.0 * std::pow(x / delta, 2) * (delta * delta + 1.0)) /
           (x * std::sqrt((delta * delta + 1.0) / (delta * delta)) * (delta + 1.0) * std::sqrt(
                                                                                         std::pow(x / delta, 2) + 1.0));
}

inline double normalized_huber_hessian(double x, double delta)
{
    const double epsilon = 1e-7;
    x = x + epsilon;

    return (
               (delta * delta + 1) * (std::pow(x / delta, 2) * std::pow(std::pow(x / delta, 2) + 1.0, 1.5) -
                                      std::pow(x / delta, 4) * std::pow(std::pow(x / delta, 2) + 1.0, 0.5))

                   ) /
           (x * x * std::sqrt((delta * delta + 1) / (delta * delta)) * (delta + 1.0) * std::pow(
                                                                                           std::pow(x / delta, 2) + 1.0, 2));
}

}  // namespace exotica

#endif  // EXOTICA_CORE_TOOLS_SPARSE_COSTS_H_
