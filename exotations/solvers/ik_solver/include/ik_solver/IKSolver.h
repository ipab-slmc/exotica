/*
 *  Created on: 15 Jul 2014
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University Of Edinburgh 
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

#ifndef IK_SOLVER_H_
#define IK_SOLVER_H_

#include <exotica/Exotica.h>
#include <exotica/Problems/UnconstrainedEndPoseProblem.h>
#include <ik_solver/IKsolverInitializer.h>
#include <iostream>
#include <fstream>

namespace exotica
{
  /**
   * \brief	IK position solver
   */
  class IKsolver: public MotionSolver, public Instantiable<IKsolverInitializer>
  {
    public:
      IKsolver();
      virtual ~IKsolver();

      virtual void Instantiate(IKsolverInitializer& init);

      virtual void Solve(Eigen::MatrixXd & solution);

      virtual void specifyProblem(PlanningProblem_ptr pointer);

      UnconstrainedEndPoseProblem_ptr& getProblem();

      Eigen::MatrixXd PseudoInverse(Eigen::MatrixXdRefConst J);

      double error;
      ros::Duration planning_time_;

      int getMaxIteration();
      int getLastIteration();

    private:
      IKsolverInitializer parameters_;

      inline void vel_solve(double & err, int t, Eigen::VectorXdRefConst q);

      void ScaleToStepSize(Eigen::VectorXdRef xd);

      UnconstrainedEndPoseProblem_ptr prob_; // Shared pointer to the planning problem.

      Eigen::MatrixXd Cinv; //!< Weight Matrices
      Eigen::MatrixXd C;
      Eigen::MatrixXd W;
      Eigen::MatrixXd Winv;
      int iterations_;
  };
  typedef std::shared_ptr<exotica::IKsolver> IKsolver_ptr;
}

#endif /* IK_SOLVER_H_ */
