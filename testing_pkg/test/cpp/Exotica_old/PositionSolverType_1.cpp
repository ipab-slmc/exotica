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

#include "testing_pkg/Exotica/PositionSolverType_1.h"

PositionSolverType_1::PositionSolverType_1(
    const exotica::OptimisationParameters_t & params)
    : PositionSolver(params), name("PositionSolverType_1"), params_ref_(
        optimisation_params_), vel_solv_ref_(vel_solver_)
{
  clearFlags();
}

bool PositionSolverType_1::solve(Eigen::VectorXdRefConst init_conf,
    Eigen::VectorXd & solution_vector)
{
  solution_vector = init_conf; //!< Just copy one into the other
  return true;
}

bool PositionSolverType_1::initDerived(tinyxml2::XMLHandle & derived_element)
{
  derived_called = true;
  if (!derived_element.FirstChildElement("string").ToElement())
  {
    return false;
  }
  else
  {
    string_element = std::string(
        derived_element.FirstChildElement("string").ToElement()->GetText());
    return true;
  }
}

REGISTER_POSITION_SOLVER_TYPE("PositionSolverType_1", PositionSolverType_1);
