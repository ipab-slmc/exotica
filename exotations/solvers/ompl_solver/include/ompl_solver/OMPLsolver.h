/*
 *  Created on: 19 Jun 2014
 *      Author: Vladimir Ivan
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

#ifndef OMPLSOLVER_H_
#define OMPLSOLVER_H_

#include <exotica/MotionSolver.h>
#include <ompl_solver/OMPLsolverInitializer.h>
#include "ompl_solver/OMPLBaseSolver.h"
#include <ros/package.h>
#include <fstream>

#define REGISTER_OMPL_SOLVER_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::MotionSolver, TYPE, DERIV)

namespace exotica
{
  class OMPLsolver: public MotionSolver, Instantiable<OMPLsolverInitializer>
  {
    public:
      OMPLsolver();

      virtual void Instantiate(OMPLsolverInitializer& init);

      /*
       * \breif Default destructor
       */
      virtual ~OMPLsolver();

      /**
       * \brief Binds the solver to a specific problem which must be pre-initalised
       * @param pointer Shared pointer to the motion planning problem
       * @return        Successful if the problem is a valid AICOProblem
       */
      virtual void specifyProblem(PlanningProblem_ptr pointer);

      void Solve(Eigen::MatrixXd & solution);

      /*
       * \brief Get planning problem
       */
      const SamplingProblem_ptr getProblem() const
      {
        return prob_;
      }

      /*
       * \brief Get planning algorithm
       */
      const std::string getAlgorithm()
      {
        return base_solver_->getPlannerName();
      }

      const OMPLBaseSolver_ptr getOMPLSolver()
      {
        return base_solver_;
      }

      virtual std::string print(std::string prepend);

      virtual void setGoalState(Eigen::VectorXdRefConst qT, const double eps = std::numeric_limits<double>::epsilon());
    private:
      SamplingProblem_ptr prob_; //!< Shared pointer to the planning problem.

      OMPLBaseSolver_ptr base_solver_;

      OMPLsolverInitializer parameters_;
  };

  typedef std::shared_ptr<exotica::OMPLsolver> OMPLsolver_ptr;

} /* namespace exotica */

#endif /* OMPLSOLVER_H_ */
