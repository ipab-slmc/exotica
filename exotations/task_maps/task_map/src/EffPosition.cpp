/*
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

#include "EffPosition.h"

REGISTER_TASKMAP_TYPE("EffPosition", exotica::EffPosition);

namespace exotica
{
  EffPosition::EffPosition()
  {
    //!< Empty constructor
  }

  void EffPosition::initialise(const rapidjson::Value& a)
  {
    std::vector<std::string> tmp_eff(1);
    getJSON(a["linkName"], tmp_eff[0]);
    Eigen::VectorXd rel;
    getJSON(a["pointInLink"], rel);
    if(rel.rows() != 3) throw_named("Incorrect size!");
    std::vector<KDL::Frame> tmp_offset(1);
    tmp_offset[0] = KDL::Frame::Identity();
    tmp_offset[0].p[0] = rel(0);
    tmp_offset[0].p[1] = rel(1);
    tmp_offset[0].p[2] = rel(2);
    scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
  }

  void EffPosition::update(Eigen::VectorXdRefConst& x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    PHI = EFFPHI;
    if (updateJacobian_)
    {
      JAC = EFFJAC;
    }
  }

  void EffPosition::initDerived(tinyxml2::XMLHandle & handle)
  {
  }

  void EffPosition::taskSpaceDim(int & task_dim)
  {
    if (!scene_)
    {
      throw_named("Kinematic scene has not been initialized!");
    }
    else
    {
      task_dim = scene_->getMapSize(object_name_) * 3;
    }
  }
}
