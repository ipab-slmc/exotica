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

// The algorithm is based on the paper :
// Chan, Tony F.; Golub, Gene H.; LeVeque, Randall J. (1979), “Updating Formulae and a Pairwise Algorithm for Computing Sample Variances.”, Technical Report STAN-CS-79-773, Department of Computer Science, Stanford University.

#ifndef EXOTICA_AICO_SOLVER_INCREMENTAL_GAUSSIAN_H_
#define EXOTICA_AICO_SOLVER_INCREMENTAL_GAUSSIAN_H_

#include <Eigen/Dense>
#include <vector>

namespace exotica
{
class SinglePassMeanCovariance
{
private:
    int D = 0;
    double W = 0;
    Eigen::VectorXd T = Eigen::VectorXd(0);
    Eigen::VectorXd dX = Eigen::VectorXd(0);
    Eigen::MatrixXd S = Eigen::MatrixXd(0, 0);

public:
    SinglePassMeanCovariance() = default;

    SinglePassMeanCovariance(int D_)
    {
        resize(D_);
    }

    void resize(int D_)
    {
        D = D_;

        T.resize(D_);
        dX.resize(D_);
        S.resize(D_, D_);

        clear();
    }

    void clear()
    {
        T.setZero();
        dX.setZero();
        S.setZero();
        W = 0.;
    }

    void add(const Eigen::Ref<const Eigen::VectorXd>& x)
    {
        if (W == 0.)
        {
            W = 1.;
            T = x;
            S.setZero();
            return;
        }
        W += 1.;
        T += x;
        double f = 1. / W / (W - 1.);
        dX = W * x - T;
        for (int r = 0; r < D; ++r)
        {
            for (int c = 0; c < D; ++c)
            {
                S(r, c) += f * dX(r) * dX(c);
            }
        }
    }

    inline void add(SinglePassMeanCovariance& M)
    {
        add(M.W, M.T, M.S);
    }

    void add(double& W_, const Eigen::Ref<const Eigen::VectorXd>& T_,
             const Eigen::Ref<const Eigen::VectorXd>& S_)
    {
        if (W == 0.)
        {
            W = W_;
            T = T_;
            S = S_;
            return;
        }
        dX = T_ / W_ - T / W;

        double f = W * W_ / (W + W_);
        for (int r = 0; r < D; ++r)
        {
            for (int c = 0; c < D; ++c)
            {
                S(r, c) += S_(r, c) + f * dX(r) * dX(c);
            }
        }
        T += T_;
        W += W_;
    }

    inline void addw(double w, const Eigen::Ref<const Eigen::VectorXd>& x)
    {
        if (W == 0.)
        {
            W = w;
            T = w * x;
            S.setZero();
            return;
        }

        dX = x - T / W;

        double f = W * w / (W + w);
        for (int r = 0; r < D; ++r)
        {
            for (int c = 0; c < D; ++c)
            {
                S(r, c) += f * dX(r) * dX(c);
            }
        }

        T += w * x;
        W += w;
    }

    void mean(Eigen::VectorXd& mu)
    {
        mu = T / W;
    }
    void cov(Eigen::MatrixXd& sig)
    {
        sig = S / W;
    }
    void covp(Eigen::MatrixXd& sig)
    {
        sig = S / (W - 1.);
    }
};
}  // namespace exotica

#endif  // EXOTICA_AICO_SOLVER_INCREMENTAL_GAUSSIAN_H_
