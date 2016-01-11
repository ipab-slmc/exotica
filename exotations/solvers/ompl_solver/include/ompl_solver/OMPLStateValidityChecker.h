/*
 *  Created on: 9 July 2014
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

#ifndef OMPLSTATEVALIDITYCHECKER_H_
#define OMPLSTATEVALIDITYCHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include "ompl_solver/OMPLsolver.h"
#include "ompl_solver/OMPLProblem.h"
#include <boost/thread/mutex.hpp>
namespace exotica
{

  class OMPLsolver;

  class OMPLStateValidityChecker: public ompl::base::StateValidityChecker
  {
    public:
      OMPLStateValidityChecker(exotica::OMPLsolver* sol);
      virtual
      ~OMPLStateValidityChecker();

      virtual bool isValid(const ompl::base::State *state) const;

      virtual bool isValid(const ompl::base::State *state, double &dist) const;

      virtual double clearance(const ompl::base::State *state) const;
    protected:
      exotica::OMPLsolver* sol_;
      boost::shared_ptr<exotica::OMPLProblem> prob_;
      bool compound_;
  };

} /* namespace exotica */

#endif /* OMPLSTATEVALIDITYCHECKER_H_ */
