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

#include "ompl_imp_solver/OMPLRNStateSpace.h"

namespace exotica
{
  OMPLRNStateSpace::OMPLRNStateSpace(unsigned int dim, const Server_ptr &server,
      OMPLProblem_ptr &prob)
      : OMPLBaseStateSpace(dim, server, prob)
  {
    setName("OMPLRNStateSpace");
    addSubspace(
        ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim)),
        1.0);
    ompl::base::RealVectorBounds bounds(dim);
    for (int i = 0; i < dim; i++)
    {
      bounds.setHigh(i, prob->getBounds()[i + dim]);
      bounds.setLow(i, prob->getBounds()[i]);
    }
    getSubspace(0)->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    lock();
  }

  ompl::base::StateSamplerPtr OMPLRNStateSpace::allocDefaultStateSampler() const
  {
    return CompoundStateSpace::allocDefaultStateSampler();
  }

  EReturn OMPLRNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q,
      ompl::base::State *state) const
  {
    if (!state)
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (q.rows() != (int) getDimension())
    {
      ERROR(
          "State vector ("<<q.rows()<<") and internal state ("<<(int)getDimension()<<") dimension disagree");
      return FAILURE;
    }
    memcpy(state->as<OMPLRNStateSpace::StateType>()->getRNSpace().values,
        q.data(), sizeof(double) * q.rows());
    return SUCCESS;
  }

  EReturn OMPLRNStateSpace::OMPLToExoticaState(const ompl::base::State *state,
      Eigen::VectorXd &q) const
  {
    if (!state)
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (q.rows() != (int) getDimension()) q.resize((int) getDimension());
    memcpy(q.data(),
        state->as<OMPLRNStateSpace::StateType>()->getRNSpace().values,
        sizeof(double) * q.rows());
    return SUCCESS;
  }
}

