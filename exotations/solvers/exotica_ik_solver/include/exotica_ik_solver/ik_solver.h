/* 
 * Copyright (c) 2016, University of Edinburgh
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

#ifndef EXOTICA_IK_SOLVER_IK_SOLVER_H_
#define EXOTICA_IK_SOLVER_IK_SOLVER_H_

#include <fstream>
#include <iostream>

#include <exotica/MotionSolver.h>
#include <exotica/Problems/UnconstrainedEndPoseProblem.h>

#include <exotica_ik_solver/IKSolverInitializer.h>

namespace exotica
{
/**
 * \brief	IK position solver
 */
class IKSolver : public MotionSolver, public Instantiable<IKSolverInitializer>
{
public:
    IKSolver();
    virtual ~IKSolver();

    void Instantiate(IKSolverInitializer& init) override;

    void Solve(Eigen::MatrixXd& solution) override;

    void specifyProblem(PlanningProblem_ptr pointer) override;

private:
    IKSolverInitializer parameters_;

    Eigen::MatrixXd PseudoInverse(Eigen::MatrixXdRefConst J);
    void ScaleToStepSize(Eigen::VectorXdRef xd);

    UnconstrainedEndPoseProblem_ptr prob_;  // Shared pointer to the planning problem.

    Eigen::MatrixXd Cinv_;  //!< Weight Matrices
    Eigen::MatrixXd C_;
    Eigen::MatrixXd W_;
    Eigen::MatrixXd Winv_;
};
}

#endif /* EXOTICA_IK_SOLVER_IK_SOLVER_H_ */
