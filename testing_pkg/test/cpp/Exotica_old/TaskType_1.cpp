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

#include "testing_pkg/Exotica/TaskType_1.h"

ExoticaTaskTest_1::ExoticaTaskTest_1(
    const exotica::OptimisationParameters_t & optimisation_params)
    : TaskDefinition(optimisation_params)
{
  name = "ExoticaTask_1";
  derived_called = false;
  update_called = false;
}

bool ExoticaTaskTest_1::updateTask(Eigen::VectorXdRefConst configuration,
    int index)
{
  update_called = true;
  if (configuration.size() != 3)
  {
    return false;
  }
  Eigen::VectorXd phi(1);
  phi
      << configuration[0] * configuration[0] + 2 * configuration[1]
          + configuration[2];
  bool success = setPhi(phi, index);
  Eigen::MatrixXd jacobian(1, 3);
  jacobian(0, 0) = 2 * configuration[0];
  jacobian(0, 1) = 2;
  jacobian(0, 2) = 1;
  success &= setJacobian(jacobian, index);
  return success;  //!< Just return true
}

bool ExoticaTaskTest_1::initDerived(tinyxml2::XMLHandle & derived_element)
{
  derived_called = true;
  return true;
}

REGISTER_TASK_TYPE("ExoticaTask_1", ExoticaTaskTest_1);
REGISTER_FOR_TESTING("ExoticaTask_1", "ExoticaTask1_default.xml");
