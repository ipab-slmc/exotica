/*
 *      Author: Michael Camilleri
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

#ifndef EXOTICA_MOTION_SOLVER_H
#define EXOTICA_MOTION_SOLVER_H

#include "exotica/Object.h"
#include "exotica/TaskMap.h"
#include "exotica/TaskDefinition.h"
#include "exotica/PlanningProblem.h"
#include "exotica/Server.h"
#include "exotica/Property.h"

#define REGISTER_MOTIONSOLVER_TYPE(TYPE, DERIV) EXOTICA_REGISTER(exotica::MotionSolver, TYPE, DERIV)

namespace exotica
{
  class MotionSolver: public Object, Uncopyable, public virtual InstantiableBase
  {
    public:
      /**
       * \brief Default constructor
       */
      MotionSolver();
      virtual ~MotionSolver()
      {
      }

      virtual void InstantiateBase(const Initializer& init);

      /**
       * \brief Binds the solver to a specific problem which must be pre-initalised
       * @param problem Shared pointer to the motion planning problem
       * @return        Always successful
       */
      virtual void specifyProblem(PlanningProblem_ptr pointer);

      void specifyProblem(PlanningProblem_ptr goals,
          PlanningProblem_ptr costs, PlanningProblem_ptr goalBias,
          PlanningProblem_ptr samplingBias);

      /**
       * \brief	Solve the problem. PURE VIRTUAL
       * @param	q0			Start configuration
       * @param	solution	Solution
       * @param	t			Time step
       */
      virtual void Solve(Eigen::VectorXdRefConst q0,
          Eigen::MatrixXd & solution) = 0;

      /*
       * \brief	Check if a problem is solvable by this solver (Pure Virtual)
       * @param	prob		Planning problem
       * @return	True if solvable, false otherwise
       */
      virtual bool isSolvable(const PlanningProblem_ptr & prob) = 0;

      virtual void setGoalState(const Eigen::VectorXd & qT, const double eps =
              std::numeric_limits<double>::epsilon()) {throw_named("Not implemented!");}
      virtual void setGoal(const std::string & task_name,
          Eigen::VectorXdRefConst goal, int t = 0) {throw_named("Not implemented!");}

      virtual void setRho(const std::string & task_name, const double rho,int t = 0) {throw_named("Not implemented!");}
      virtual void getGoal(const std::string & task_name, Eigen::VectorXd& goal,int t = 0) {throw_named("Not implemented!");}
      virtual void getRho(const std::string & task_name, double& rho, int t = 0) {throw_named("Not implemented!");}

      PlanningProblem_ptr getProblem() {return problem_;}

      virtual std::string print(std::string prepend);
    protected:
      /**
       * \brief Derived-elements initialiser: Pure Virtual
       * @param handle XMLHandle to the Solver element
       * @return       Should indicate success or otherwise
       */

      PlanningProblem_ptr problem_; //!< Shared pointer to the planning problem: Anyone using it should check if it is initialised
      Server_ptr server_; //!< Pointer to EXOTica parameter server;
  };

  typedef exotica::Factory<exotica::MotionSolver> MotionSolver_fac;
  typedef boost::shared_ptr<exotica::MotionSolver> MotionSolver_ptr;
}

#endif
