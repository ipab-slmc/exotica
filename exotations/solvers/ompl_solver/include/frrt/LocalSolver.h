/*
 * LocalSolver.h
 *
 *  Created on: 2 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_LOCALSOLVER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_LOCALSOLVER_H_

#include <ik_solver/ik_solver.h>

namespace exotica
{
  ///	\brief	Implementation of FRRTs local solver
  class LocalSolver: public IKsolver
  {
    public:
      /*
       * \brief	Constructor
       */
      LocalSolver();

      /*
       * \brief	Destructor
       */
      ~LocalSolver();

      EReturn localSolve();

  };
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_LOCALSOLVER_H_ */
