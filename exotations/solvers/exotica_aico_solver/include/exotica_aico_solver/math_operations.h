/*
 *  Created on: 13 Mar 2018
 *      Author: Wolfgang Merkt, Vladimir Ivan
 *
 *  This code is based on algorithm developed by Marc Toussaint
 *  M. Toussaint: Robot Trajectory Optimization using Approximate Inference. In Proc. of the Int. Conf. on Machine Learning (ICML 2009), 1049-1056, ACM, 2009.
 *  http://ipvs.informatik.uni-stuttgart.de/mlr/papers/09-toussaint-ICML.pdf
 *  Original code available at http://ipvs.informatik.uni-stuttgart.de/mlr/marc/source-code/index.html
 *
 * 
 * Copyright (c) 2018, University of Edinburgh
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

#ifndef EXOTICA_AICO_SOLVER_MATH_OPERATIONS_H_
#define EXOTICA_AICO_SOLVER_MATH_OPERATIONS_H_

#include <exotica/Exotica.h>

#include "f2c.h"
#include "lapack/cblas.h"
#undef small
#undef large
#include <lapack/clapack.h>
#undef double
#undef max
#undef min
#undef abs

namespace exotica
{
/**
 * \brief Computes an inverse of symmetric positive definite matrix using LAPACK (very fast)
 * @param Ainv Resulting inverted matrix.
 * @param A A symmetric positive definite matrix to be inverted.
 */
void inverseSymPosDef(Eigen::Ref<Eigen::MatrixXd> Ainv_,
                      const Eigen::Ref<const Eigen::MatrixXd>& A_);

/**
 * \brief Computes the solution to the linear problem \f$x=Ab\f$ for symmetric positive definite matrix A
 */
void AinvBSymPosDef(Eigen::Ref<Eigen::VectorXd> x_,
                    const Eigen::Ref<const Eigen::MatrixXd>& A_,
                    const Eigen::Ref<const Eigen::VectorXd>& b_,
                    Eigen::MatrixXd& linSolverTmp,
                    int n_in);
}

#endif  // EXOTICA_AICO_SOLVER_MATH_OPERATIONS_H_
