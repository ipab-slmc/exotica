/*
 *  Created on: 2 Mar 2016
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLRNSTATESPACE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLRNSTATESPACE_H_

#include "ompl_imp_solver/OMPLBaseStateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>

namespace exotica
{
  class OMPLRNStateSpace: public OMPLBaseStateSpace
  {
    public:
      class StateType: public ob::CompoundStateSpace::StateType
      {
        public:
          StateType()
              : CompoundStateSpace::StateType()
          {

          }

          const ob::RealVectorStateSpace::StateType & getRNSpace() const
          {
            return *as<ob::RealVectorStateSpace::StateType>(0);
          }

          ob::RealVectorStateSpace::StateType & getRNSpace()
          {
            return *as<ob::RealVectorStateSpace::StateType>(0);
          }
      };
      OMPLRNStateSpace(unsigned int dim, const Server_ptr &server,
          OMPLProblem_ptr &prob);

      virtual ompl::base::StateSamplerPtr allocDefaultStateSampler(
          const ompl::base::StateSpace *ss);
      virtual EReturn ExoticaToOMPLState(const Eigen::VectorXd &q,
          ompl::base::State *state) const;
      virtual EReturn OMPLToExoticaState(const ompl::base::State *state,
          Eigen::VectorXd &q) const;
  };
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLRNSTATESPACE_H_ */
