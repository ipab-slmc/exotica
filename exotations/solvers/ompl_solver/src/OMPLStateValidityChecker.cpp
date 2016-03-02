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

#include "ompl_solver/OMPLStateValidityChecker.h"

namespace exotica
{

  OMPLStateValidityChecker::OMPLStateValidityChecker(exotica::OMPLsolver* sol)
      : sol_(sol), ompl::base::StateValidityChecker(
          sol->getOMPLSimpleSetup()->getSpaceInformation()), prob_(
          sol->getProblem())
  {
    compound_ = prob_->isCompoundStateSpace();
  }

  OMPLStateValidityChecker::~OMPLStateValidityChecker()
  {
  }

  bool OMPLStateValidityChecker::isValid(const ompl::base::State *state) const
  {
    double tmp;
    return isValid(state, tmp);
  }

  bool OMPLStateValidityChecker::isValid(const ompl::base::State *state,
      double &dist) const
  {
    Eigen::VectorXd q(prob_->getSpaceDim());
    if (compound_)
      boost::static_pointer_cast<OMPLSE3RNStateSpace>(
          sol_->getOMPLStateSpace())->OMPLStateToEigen(state, q);
    else
      boost::static_pointer_cast<OMPLStateSpace>(sol_->getOMPLStateSpace())->copyFromOMPLState(
          state, q);

    {
      boost::mutex::scoped_lock lock(prob_->getLock());
      prob_->update(q, 0);

// TODO: Implement this
// Constraints
// State rejection (e.g. collisions)
      for (auto & it : prob_->scenes_)
      {
        if (!it.second->getCollisionScene()->isStateValid())
        {
          dist = -1;
          return false;
        }
      }

      double err;
      bool terminate;
      for (TaskTerminationCriterion_ptr goal : prob_->getGoals())
      {
        if (goal->object_name_.compare("CoMTask") == 0)
        {
          goal->terminate(terminate, err);
          if (!terminate)
          {
            dist = -1;
            return false;
          }
        }
      }

    }

    return true;
  }

  double OMPLStateValidityChecker::clearance(
      const ompl::base::State *state) const
  {
    double tmp;
    isValid(state, tmp);
    return tmp;
  }
} /* namespace exotica */
