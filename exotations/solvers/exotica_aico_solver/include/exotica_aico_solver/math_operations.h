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

#ifndef EXOTICA_AICO_SOLVER_MATH_OPERATIONS_H_
#define EXOTICA_AICO_SOLVER_MATH_OPERATIONS_H_

#include <Eigen/Cholesky>
#include <Eigen/Dense>

namespace exotica
{
/// \brief Computes an inverse of a symmetric positive definite matrix.
/// @param Ainv Resulting inverted matrix (using Cholesky factorization).
/// @param A A symmetric positive definite matrix to be inverted.
template <typename T1, typename T2>
static inline void inverseSymPosDef(T1& Ainv, const T2& A)
{
    Ainv = A.llt().solve(Eigen::MatrixXd::Identity(A.rows(), A.cols()));
}

/// \brief Computes the solution to the linear problem \f$x=Ab\f$ for symmetric positive definite matrix A
template <typename T1, typename T2, typename T3>
static inline void AinvBSymPosDef(T1& x, const T2& A, const T3& b)
{
    x = A.llt().solve(b);
}
}  // namespace exotica

#endif  // EXOTICA_AICO_SOLVER_MATH_OPERATIONS_H_
