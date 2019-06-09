//
// Copyright (c) 2019, University of Edinburgh
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

#ifndef EXOTICA_DDP_SOLVER_BOX_QP_H
#define EXOTICA_DDP_SOLVER_BOX_QP_H

#include <Eigen/Dense>

namespace exotica
{
    typedef struct BoxQPSolution
    {
        Eigen::MatrixXd Hff_inv;
        Eigen::MatrixXd x;
        std::vector<int> free_idx; 
        std::vector<int> clamped_idx; 
    } BoxQPSolution;

    inline BoxQPSolution BoxQP(Eigen::MatrixXd H, Eigen::VectorXd q,
        Eigen::VectorXd b_low, Eigen::VectorXd b_high, Eigen::VectorXd x_init, const double gamma,
        const int max_iterations, const double epsilon)
    {
        int it = 0;
        Eigen::VectorXd delta_xf, x = x_init;
        std::vector<int> clamped, free;
        Eigen::VectorXd grad = q + H * x_init;
        Eigen::MatrixXd Hff, Hfc, Hff_inv;

        Hff_inv = (Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-5 + H).inverse();

        if (grad.norm() <= epsilon)
            return { Hff_inv, x_init, free, clamped };

        while (grad.norm() > epsilon && it < max_iterations)
        {
            ++it;
            grad = q + H * x;
            clamped.clear();
            free.clear();

            for (int i = 0; i < grad.size(); ++i)
            {
                if ((x(i) == b_low(i) && grad(i) > 0) || (x(i) == b_high(i) && grad(i) < 0))
                    clamped.push_back(i);
                else free.push_back(i);
            }
            
            if (free.size() == 0)
                return { Hff_inv, x, free, clamped };

            Hff.resize(free.size(), free.size());
            Hfc.resize(free.size(), clamped.size());

            if (clamped.size() == 0)
                Hff = H;
            else
            {
                for (int i = 0; i < free.size(); ++ i)
                    for (int j = 0; j < free.size(); ++ j)
                        Hff(i, j) = H(free[i], free[j]);

                for (int i = 0; i < free.size(); ++ i)
                    for (int j = 0; j < clamped.size(); ++ j)
                        Hfc(i, j) = H(free[i], clamped[j]);
            }
        
            // NOTE: Array indexing not supported in current eigen version
            Eigen::VectorXd q_free(free.size()), x_free(free.size()), x_clamped(clamped.size());
            for (int i = 0; i < free.size(); ++ i)
            {
                q_free(i) = q(free[i]);
                x_free(i) = x(free[i]);
            }

            for (int j = 0; j < clamped.size(); ++ j)
                x_clamped(j) = x(clamped[j]);

            Hff_inv = (Eigen::MatrixXd::Identity(Hff.rows(), Hff.cols()) * 1e-5 + Hff).inverse();

            if (clamped.size() == 0)
                delta_xf = -Hff_inv * (q_free) - x_free;
            else
                delta_xf = -Hff_inv * (q_free + Hfc * x_clamped) - x_free;

            double f_old = (
                0.5 * x.transpose() * H * x + q.transpose() * x
            )(0);
            const Eigen::VectorXd alpha_space = Eigen::VectorXd::LinSpaced(10, 1.0, 0.1);

            bool armijo_reached = false;
            Eigen::VectorXd x_new;
            for (int ai = 0; ai < alpha_space.rows(); ++ai)
            {
                int alpha = alpha_space[ai];

                x_new = x;
                for (int i = 0; i < free.size(); ++ i)
                    x_new(free[i]) = std::max(std::min(
                        x(free[i]) + alpha * delta_xf(i), b_high(i)
                        ), b_low(i)
                    );

                double f_new = (
                    0.5 * x_new.transpose() * H * x_new + q.transpose() * x_new
                )(0);
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

        return { Hff_inv, x, free, clamped };
    }

    inline BoxQPSolution BoxQP(Eigen::MatrixXd H, Eigen::VectorXd q,
        Eigen::VectorXd b_low, Eigen::VectorXd b_high, Eigen::VectorXd x_init)
        {
            constexpr double epsilon = 1e-5;
            constexpr double gamma = 0.1;
            constexpr int max_iterations = 100;
            return BoxQP(H, q, b_low, b_high, x_init, gamma, max_iterations, epsilon);
        }
} // namespace exotica

#endif
