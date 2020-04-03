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

#ifndef EXOTICA_CORE_BOX_QP_OLD_H_
#define EXOTICA_CORE_BOX_QP_OLD_H_

#include <Eigen/Dense>
#include <vector>

#include "exotica_core/tools/box_qp.h"

namespace exotica
{
inline BoxQPSolution ExoticaBoxQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& b_low, const Eigen::VectorXd& b_high,
                                  const Eigen::VectorXd& x_init, const double gamma,
                                  const int max_iterations, const double epsilon, const double lambda,
                                  bool use_polynomial_linesearch = false,
                                  bool use_cholesky_factorization = false)
{
    int it = 0;
    Eigen::VectorXd delta_xf, x = x_init;
    std::vector<size_t> clamped_idx, free_idx;
    Eigen::VectorXd grad = q + H * x_init;
    Eigen::MatrixXd Hff, Hfc, Hff_inv;

    std::vector<double> alphas_;
    const std::size_t& n_alphas_ = 10;
    alphas_.resize(n_alphas_);
    const Eigen::VectorXd alphas_linear = Eigen::VectorXd::LinSpaced(10, 1.0, 0.1);
    for (std::size_t n = 0; n < n_alphas_; ++n)
    {
        if (use_polynomial_linesearch)
        {
            alphas_[n] = 1. / pow(2., static_cast<double>(n));
        }
        else
        {
            alphas_[n] = alphas_linear(n);
        }
    }

    // Ensure a feasible warm-start
    for (int i = 0; i < x.size(); ++i)
    {
        x(i) = std::max(std::min(x_init(i), b_high(i)), b_low(i));
    }

    if (use_cholesky_factorization)
    {
        Hff_inv = (Eigen::MatrixXd::Identity(H.rows(), H.cols()) * lambda + H).llt().solve(Eigen::MatrixXd::Identity(H.rows(), H.cols()));
    }
    else
    {
        Hff_inv = (Eigen::MatrixXd::Identity(H.rows(), H.cols()) * lambda + H).inverse();
    }

    if (grad.norm() <= epsilon)
    {
        for (int i = 0; i < x.size(); ++i) free_idx.push_back(i);
        return {Hff_inv, x_init, free_idx, clamped_idx};
    }

    while (grad.norm() > epsilon && it < max_iterations)
    {
        ++it;
        grad = q + H * x;
        clamped_idx.clear();
        free_idx.clear();

        for (int i = 0; i < grad.size(); ++i)
        {
            if ((x(i) == b_low(i) && grad(i) > 0) || (x(i) == b_high(i) && grad(i) < 0))
                clamped_idx.push_back(i);
            else
                free_idx.push_back(i);
        }

        if (free_idx.size() == 0)
            break;

        Hff.resize(free_idx.size(), free_idx.size());
        Hfc.resize(free_idx.size(), clamped_idx.size());

        if (clamped_idx.size() == 0)
            Hff = H;
        else
        {
            for (size_t i = 0; i < free_idx.size(); ++i)
                for (size_t j = 0; j < free_idx.size(); ++j)
                    Hff(i, j) = H(free_idx[i], free_idx[j]);

            for (size_t i = 0; i < free_idx.size(); ++i)
                for (size_t j = 0; j < clamped_idx.size(); ++j)
                    Hfc(i, j) = H(free_idx[i], clamped_idx[j]);
        }

        // NOTE: Array indexing not supported in current eigen version
        Eigen::VectorXd q_free(free_idx.size()), x_free(free_idx.size()), x_clamped(clamped_idx.size());
        for (size_t i = 0; i < free_idx.size(); ++i)
        {
            q_free(i) = q(free_idx[i]);
            x_free(i) = x(free_idx[i]);
        }

        for (size_t j = 0; j < clamped_idx.size(); ++j)
            x_clamped(j) = x(clamped_idx[j]);

        // Compute the inverse
        Hff.diagonal().array() += lambda;
        if (use_cholesky_factorization)
        {
            Hff_inv = Hff.llt().solve(Eigen::MatrixXd::Identity(Hff.rows(), Hff.cols()));
        }
        else
        {
            Hff_inv = Hff.inverse();
        }

        if (clamped_idx.size() == 0)
            delta_xf = -Hff_inv * (q_free)-x_free;
        else
            delta_xf = -Hff_inv * (q_free + Hfc * x_clamped) - x_free;

        double f_old = (0.5 * x.transpose() * H * x + q.transpose() * x)(0);

        bool armijo_reached = false;
        Eigen::VectorXd x_new;
        for (std::size_t ai = 0; ai < alphas_.size(); ++ai)
        {
            const double& alpha = alphas_[ai];

            x_new = x;
            for (size_t i = 0; i < free_idx.size(); ++i)
                x_new(free_idx[i]) = std::max(std::min(
                                                  x(free_idx[i]) + alpha * delta_xf(i), b_high(i)),
                                              b_low(i));

            double f_new = (0.5 * x_new.transpose() * H * x_new + q.transpose() * x_new)(0);
            Eigen::VectorXd x_diff = x - x_new;

            // armijo criterion>
            double armijo_coef = (f_old - f_new) / (grad.transpose() * x_diff + 1e-5);
            if (armijo_coef > gamma)
            {
                armijo_reached = true;
                x = x_new;
                break;
            }
        }

        // break if no step made
        if (!armijo_reached) break;
    }

    return {Hff_inv, x, free_idx, clamped_idx};
}

inline BoxQPSolution ExoticaBoxQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& b_low, const Eigen::VectorXd& b_high,
                                  const Eigen::VectorXd& x_init)
{
    constexpr double epsilon = 1e-5;
    constexpr double gamma = 0.1;
    constexpr int max_iterations = 100;
    constexpr double lambda = 1e-5;
    return ExoticaBoxQP(H, q, b_low, b_high, x_init, gamma, max_iterations, epsilon, lambda, false, false);
}
}  // namespace exotica

#endif  // EXOTICA_CORE_BOX_QP_OLD_H_
