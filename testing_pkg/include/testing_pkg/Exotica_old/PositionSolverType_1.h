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

#ifndef TESTING_POSITION_SOLVER_TYPE_1_H
#define TESTING_POSITION_SOLVER_TYPE_1_H

#include <exotica/PositionSolver.h>
#include <exotica/PosSolverFactory.h>

#include "testing_pkg/TestingTools.h"

class PositionSolverType_1: public exotica::PositionSolver
{
  public:

    PositionSolverType_1(const exotica::OptimisationParameters_t & params);

    virtual bool solve(Eigen::VectorXdRefConst init_conf,
        Eigen::VectorXd & solution_vector);

    void clearFlags()
    {
      derived_called = false;
    }

    bool derived_called;
    const std::string name;
    std::string string_element;
    const exotica::OptimisationParameters_t & params_ref_; //!< Reference to the underlying paramaters (for testing)
    boost::shared_ptr<exotica::VelocitySolver> & vel_solv_ref_;

  protected:

    virtual bool initDerived(tinyxml2::XMLHandle & derived_element);
};

#endif
