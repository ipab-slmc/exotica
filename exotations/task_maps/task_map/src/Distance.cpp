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

#include "Distance.h"

REGISTER_TASKMAP_TYPE("Distance", exotica::Distance);

namespace exotica
{
  Distance::Distance()
  {
    //!< Empty constructor
  }

  void Distance::initialise(const rapidjson::Value& a)
  {
    std::string eff;
    getJSON(a["linkName"], eff);
    std::vector<std::string> tmp_eff(2);
    std::vector<KDL::Frame> tmp_offset(2);
    Eigen::VectorXd rel;
    getJSON(a["pointInLink"], rel);
    if (rel.rows() != 3) throw_named("Incorrect size!");
    ok(getJSON(a["referenceFrame"], tmp_offset[1]);
    tmp_offset[0] = KDL::Frame(KDL::Vector(rel(0), rel(1), rel(2)));
    tmp_offset[1].p = tmp_offset[1].p - scene_->getSolver().getRootOffset().p;
    tmp_eff[0] = eff;
    tmp_eff[1] = getScene()->getRootName();
    ref_pose_ = tmp_offset[1];
    scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
  }

  void Distance::update(Eigen::VectorXdRefConst x, const int t)
  {
    if (!isRegistered(t) || !getEffReferences())
    {
      throw_named("Not fully initialized!");
    }
    JAC.setZero();
    for (int i = 0; i < PHI.rows(); i++)
    {
      PHI(i) = sqrt(
          (EFFPHI(i * 2 * 3) - EFFPHI(i * 2 * 3 + 3))
              * (EFFPHI(i * 2 * 3) - EFFPHI(i * 2 * 3 + 3))
              + (EFFPHI(i * 2 * 3 + 1) - EFFPHI(i * 2 * 3 + 4))
                  * (EFFPHI(i * 2 * 3 + 1) - EFFPHI(i * 2 * 3 + 4))
              + (EFFPHI(i * 2 * 3 + 2) - EFFPHI(i * 2 * 3 + 5))
                  * (EFFPHI(i * 2 * 3 + 2) - EFFPHI(i * 2 * 3 + 5)));

      if (updateJacobian_ && PHI(i) > 1e-50)
      {
        for (int j = 0; j < JAC.cols(); j++)
        {
          JAC(i, j) = ((EFFPHI(i * 2 * 3) - EFFPHI(i * 2 * 3 + 3))
              * (EFFJAC(i * 2 * 3, j) - EFFJAC(i * 2 * 3 + 3, j))
              + (EFFPHI(i * 2 * 3 + 1) - EFFPHI(i * 2 * 3 + 4))
                  * (EFFJAC(i * 2 * 3 + 1, j) - EFFJAC(i * 2 * 3 + 4, j))
              + (EFFPHI(i * 2 * 3 + 2) - EFFPHI(i * 2 * 3 + 5))
                  * (EFFJAC(i * 2 * 3 + 2, j) - EFFJAC(i * 2 * 3 + 5, j)))
              / PHI(i);
        }
      }
    }
  }

  void Distance::initDerived(tinyxml2::XMLHandle & handle)
  {
    if (scene_->getMapSize(object_name_) % 2 != 0)
    {
      throw_named("Kinematic scene must have even number of end-effectors!");
    }
  }

  void Distance::taskSpaceDim(int & task_dim)
  {
    if (!scene_)
    {
      throw_named("Kinematic scene has not been initialized!");
    }
    else
    {
      task_dim = scene_->getMapSize(object_name_) / 2;
    }
  }
}
