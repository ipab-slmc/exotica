/*
 *  Created on: 9 Jul 2014
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

#include "ompl_solver/OMPLStateSpace.h"

namespace exotica
{

  OMPLStateSpace::OMPLStateSpace(unsigned int dim)
      : ompl::base::RealVectorStateSpace(dim)
  {
    setName("OMPLStateSpace");
  }

  boost::shared_ptr<OMPLStateSpace> OMPLStateSpace::FromProblem(
      OMPLProblem_ptr prob)
  {
    int n = prob->getSpaceDim();
    boost::shared_ptr<OMPLStateSpace> ret;
    if (n <= 0)
    {
      ERROR("State space size error!");
    }
    else
    {
      ret.reset(new OMPLStateSpace(n));
      ompl::base::RealVectorBounds bounds(n);
      if (prob->getBounds().size() == 2 * n)
      {
        for (int i = 0; i < n; i++)
        {
          bounds.setHigh(i, prob->getBounds()[i + n]);
          bounds.setLow(i, prob->getBounds()[i]);
        }
      }
      else
      {
        WARNING(
            "State space bounds were not specified!\n"<< prob->getBounds().size() << " " << n);
      }
      ret->setBounds(bounds);
    }
    return ret;
  }

  OMPLStateSpace::~OMPLStateSpace()
  {
    // TODO Auto-generated destructor stub
  }

  EReturn OMPLStateSpace::copyToOMPLState(ompl::base::State *state,
      Eigen::VectorXd q) const
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
    memcpy(state->as<ompl::base::RealVectorStateSpace::StateType>()->values,
        q.data(), sizeof(double) * q.rows());
    return SUCCESS;
  }

  EReturn OMPLStateSpace::copyFromOMPLState(const ompl::base::State *state,
      Eigen::VectorXd& q) const
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
    memcpy(q.data(),
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values,
        sizeof(double) * q.rows());
    return SUCCESS;
  }

} /* namespace exotica */
