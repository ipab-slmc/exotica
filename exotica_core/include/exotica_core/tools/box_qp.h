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

#ifndef EXOTICA_CORE_BOX_QP_H_
#define EXOTICA_CORE_BOX_QP_H_

#include <exotica_core/tools/exception.h>
#include <Eigen/Dense>
#include <vector>

namespace exotica
{
typedef struct BoxQPSolution
{
    Eigen::MatrixXd Hff_inv;
    Eigen::VectorXd x;
    std::vector<size_t> free_idx;
    std::vector<size_t> clamped_idx;
} BoxQPSolution;

inline BoxQPSolution BoxQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& q, const Eigen::VectorXd& b_low, const Eigen::VectorXd& b_high, const Eigen::VectorXd& x_init, const double th_acceptstep, const int max_iterations, const double th_gradient_tolerance, const double lambda, bool use_polynomial_linesearch = true, bool use_cholesky_factorization = true)
{
    if (lambda < 0.) ThrowPretty("lambda needs to be positive.");

    // gamma = acceptance threshold
    // epsilon = gradient tolerance
    // lambda = regularization for Cholesky factorization
    const std::size_t nx = x_init.size();

    Eigen::VectorXd delta_xf(nx), x = x_init;
    std::vector<size_t> clamped_idx, free_idx;
    Eigen::VectorXd grad(nx);
    Eigen::MatrixXd Hff(nx, nx), Hfc(nx, nx);

    std::vector<double> alphas_;
    const std::size_t& n_alphas_ = 10;
    alphas_.resize(n_alphas_);
    const Eigen::VectorXd alphas_linear = Eigen::VectorXd::LinSpaced(n_alphas_, 1.0, 0.1);
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
    double fold_, fnew_;

    // Ensure a feasible warm-start
    for (int i = 0; i < x.size(); ++i)
    {
        x(i) = std::max(std::min(x_init(i), b_high(i)), b_low(i));
    }

    BoxQPSolution solution;
    solution.clamped_idx.reserve(nx);
    solution.free_idx.reserve(nx);
    std::size_t num_free, num_clamped;
    Eigen::LLT<Eigen::MatrixXd> Hff_inv_llt_;
    Eigen::VectorXd qf_, xf_, xc_, dxf_, dx_(nx);

    for (int k = 0; k < max_iterations; ++k)
    {
        solution.clamped_idx.clear();
        solution.free_idx.clear();

        // Compute the gradient
        grad = q;
        grad.noalias() += H * x;

        // Check if any element is at the limits
        for (std::size_t i = 0; i < nx; ++i)
        {
            if ((x(i) == b_low(i) && grad(i) > 0.) || (x(i) == b_high(i) && grad(i) < 0.))
                solution.clamped_idx.push_back(i);
            else
                solution.free_idx.push_back(i);
        }
        num_free = solution.free_idx.size();
        num_clamped = solution.clamped_idx.size();

        // Check convergence
        //  a) Either norm of gradient is below threshold
        //    OR
        //  b) None of the dimensions is free (all are at boundary)
        if (grad.lpNorm<Eigen::Infinity>() <= th_gradient_tolerance || num_free == 0)
        {
            // During first iteration return the inverse of the free Hessian
            if (k == 0 && num_free != 0)
            {
                Hff.resize(num_free, num_free);
                for (std::size_t i = 0; i < num_free; ++i)
                {
                    const std::size_t& fi = solution.free_idx[i];
                    for (std::size_t j = 0; j < num_free; ++j)
                    {
                        Hff(i, j) = H(fi, solution.free_idx[j]);
                    }
                }
                // Add regularization
                if (lambda != 0.)
                {
                    Hff.diagonal().array() += lambda;
                }

                // Compute the inverse
                if (use_cholesky_factorization)
                {
                    Hff_inv_llt_.compute(Hff);
                    const Eigen::ComputationInfo& info = Hff_inv_llt_.info();
                    if (info != Eigen::Success)
                    {
                        ThrowPretty("Error during Cholesky decomposition of Hff");
                    }
                    solution.Hff_inv.setIdentity(num_free, num_free);
                    Hff_inv_llt_.solveInPlace(solution.Hff_inv);
                }
                else
                {
                    solution.Hff_inv = Hff.inverse();
                }
            }

            if (num_free == 0)
            {
                solution.Hff_inv.resize(num_free, num_free);
            }

            // Set solution
            solution.x = x;
            return solution;
        }

        // Compute the search direction as Newton step along the free space
        qf_.resize(num_free);
        xf_.resize(num_free);
        xc_.resize(num_clamped);
        dxf_.resize(num_free);
        Hff.resize(num_free, num_free);
        Hfc.resize(num_free, num_clamped);
        for (std::size_t i = 0; i < num_free; ++i)
        {
            const std::size_t& fi = solution.free_idx[i];
            qf_(i) = q(fi);
            xf_(i) = x(fi);
            for (std::size_t j = 0; j < num_free; ++j)
            {
                Hff(i, j) = H(fi, solution.free_idx[j]);
            }
            for (std::size_t j = 0; j < num_clamped; ++j)
            {
                const std::size_t cj = solution.clamped_idx[j];
                xc_(j) = x(cj);
                Hfc(i, j) = H(fi, cj);
            }
        }
        if (lambda != 0.)
        {
            Hff.diagonal().array() += lambda;
        }

        // Compute the inverse
        if (use_cholesky_factorization)
        {
            Hff_inv_llt_.compute(Hff);
            const Eigen::ComputationInfo& info = Hff_inv_llt_.info();
            if (info != Eigen::Success)
            {
                ThrowPretty("Error during Cholesky decomposition of Hff (iter=" << k << "):\n"
                                                                                << Hff << "\n"
                                                                                << "H:\n"
                                                                                << H << "\nnum_free: " << num_free << " num_clamped: " << num_clamped << " lambda: " << lambda);
            }
            solution.Hff_inv.setIdentity(num_free, num_free);
            Hff_inv_llt_.solveInPlace(solution.Hff_inv);
        }
        else
        {
            solution.Hff_inv = Hff.inverse();
        }

        dxf_ = -qf_;
        if (num_clamped != 0)
        {
            dxf_.noalias() -= Hfc * xc_;
        }
        if (use_cholesky_factorization)
        {
            Hff_inv_llt_.solveInPlace(dxf_);
        }
        else
        {
            dxf_ = solution.Hff_inv * dxf_;
        }
        dxf_ -= xf_;
        dx_.setZero();
        for (std::size_t i = 0; i < num_free; ++i)
        {
            dx_(solution.free_idx[i]) = dxf_(i);
        }

        // Try different step lengths
        Eigen::VectorXd xnew_(nx);
        fold_ = 0.5 * x.dot(H * x) + q.dot(x);
        for (std::vector<double>::const_iterator it = alphas_.begin(); it != alphas_.end(); ++it)
        {
            double steplength = *it;
            for (std::size_t i = 0; i < nx; ++i)
            {
                xnew_(i) = std::max(std::min(x(i) + steplength * dx_(i), b_high(i)), b_low(i));
            }
            fnew_ = 0.5 * xnew_.dot(H * xnew_) + q.dot(xnew_);
            if (fold_ - fnew_ > th_acceptstep * grad.dot(x - xnew_))
            {
                x = xnew_;
                break;
            }

            // If line-search fails, return.
            if (it == alphas_.end() - 1)
            {
                solution.x = x;
                return solution;
            }
        }
    }

    solution.x = x;
    return solution;
}

inline BoxQPSolution BoxQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& q,
                           const Eigen::VectorXd& b_low, const Eigen::VectorXd& b_high,
                           const Eigen::VectorXd& x_init)
{
    constexpr double epsilon = 1e-5;
    constexpr double gamma = 0.1;
    constexpr int max_iterations = 100;
    constexpr double lambda = 1e-5;
    return BoxQP(H, q, b_low, b_high, x_init, gamma, max_iterations, epsilon, lambda, true, true);
}
}  // namespace exotica

#endif  // EXOTICA_CORE_BOX_QP_H_
