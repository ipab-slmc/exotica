/*
 *  Created on: 19 Jun 2014
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

#include <exotica/Problems/SamplingProblem.h>
#include <exotica/Setup.h>

REGISTER_PROBLEM_TYPE("SamplingProblem", exotica::SamplingProblem)

namespace exotica
{

  SamplingProblem::SamplingProblem()
      : space_dim_(0)
  {
    Flags = KIN_FK;
  }

  SamplingProblem::~SamplingProblem()
  {
    // TODO Auto-generated destructor stub
  }

  std::vector<double>& SamplingProblem::getBounds()
  {
    return bounds_;
  }

  void SamplingProblem::Instantiate(SamplingProblemInitializer& init)
  {
      Parameters = init;

      if(init.LocalPlannerConfig!="")
      {
          local_planner_config_ = init.LocalPlannerConfig;
      }

      space_dim_ = scene_->getNumJoints();

      goal_ = init.Goal;

      if (scene_->getBaseType() != exotica::BASE_TYPE::FIXED)
        compound_ = true;
      else
        compound_ = false;
      std::vector<std::string> jnts;
      scene_->getJointNames(jnts);

      getBounds().resize(jnts.size() * 2);
      std::map<std::string, std::vector<double>> joint_limits = scene_->getSolver().getUsedJointLimits();
      for (int i = 0; i < jnts.size(); i++)
      {
        getBounds()[i] = joint_limits.at(jnts[i])[0];
        getBounds()[i + jnts.size()] = joint_limits.at(jnts[i])[1];
      }
  }

  void SamplingProblem::setGoalState(Eigen::VectorXdRefConst qT)
  {
      goal_ = qT;
  }

  void SamplingProblem::Update(Eigen::VectorXdRefConst x)
  {
      scene_->Update(x);
  }

  int SamplingProblem::getSpaceDim()
  {
    return space_dim_;
  }

  bool SamplingProblem::isCompoundStateSpace()
  {
    return compound_;
  }

} /* namespace exotica */
