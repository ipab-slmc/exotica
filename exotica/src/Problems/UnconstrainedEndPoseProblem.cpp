/*
 *  Created on: 15 Jul 2014
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

#include <exotica/Problems/UnconstrainedEndPoseProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("UnconstrainedEndPoseProblem", exotica::UnconstrainedEndPoseProblem)

#define XML_CHECK(x) {xmltmp=handle.FirstChildElement(x).ToElement();if (!xmltmp) throw_named("XML element '"<<x<<"' does not exist!");}

namespace exotica
{
  UnconstrainedEndPoseProblem::UnconstrainedEndPoseProblem()
      : tau_(0.01)
  {

  }

  UnconstrainedEndPoseProblem::~UnconstrainedEndPoseProblem()
  {
    //TODO
  }

  void UnconstrainedEndPoseProblem::Instantiate(UnconstrainedEndPoseProblemInitializer& init)
  {
      tau_ = init.Tolerance;
      config_w_ = Eigen::MatrixXd::Identity(init.W.rows(), init.W.rows());
      config_w_.diagonal() = init.W;
      T_ = init.T;
      for (auto& it : task_defs_)
      {
          it.second->setTimeSteps(T_);
      }
  }

  int UnconstrainedEndPoseProblem::getT()
  {
    return T_;
  }

  Eigen::MatrixXd UnconstrainedEndPoseProblem::getW()
  {
    return config_w_;
  }

  double UnconstrainedEndPoseProblem::getTau()
  {
    return tau_;
  }

  void UnconstrainedEndPoseProblem::setTau(double tau)
  {
    tau_ = tau;
  }
}

