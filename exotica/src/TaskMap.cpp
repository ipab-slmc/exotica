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

#include "exotica/TaskMap.h"
#include "exotica/PlanningProblem.h"
#include <boost/algorithm/string.hpp>

namespace exotica
{

  TaskMap::TaskMap()
      : updateJacobian_(true)
  {

  }

  Scene_ptr TaskMap::getScene()
  {
    return scene_;
  }

  std::string TaskMap::print(std::string prepend)
  {
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Scene:";
    ret += "\n" + scene_->print(prepend + "    ");
    return ret;
  }

  EReturn TaskMap::initialise(const rapidjson::Value& a)
  {
    ERROR("This has to be implemented in the derived class!");
    return FAILURE;
  }

  EReturn TaskMap::initialise(const rapidjson::Value& a, Server_ptr & server,
      const Scene_map & scene_ptr, PlanningProblem_ptr prob)
  {
    if (ok(getJSON(a["class"], object_name_)))
    {
      if (!server)
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
      std::vector<std::pair<std::string,std::string> > tmp;
      initialiseManual(object_name_, server, scene_ptr, prob, tmp);
      return initialise(a);
    }
    else
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
  }

  EReturn TaskMap::initialiseManual(std::string name, Server_ptr & server,
      const Scene_map & scene_ptr, boost::shared_ptr<PlanningProblem> prob,
      std::vector<std::pair<std::string,std::string> >& params)
  {
    object_name_ = name + std::to_string((unsigned long) this);
    server_ = server;
    scene_ = scene_ptr.begin()->second;
    poses = prob->poses;
    posesJointNames = prob->posesJointNames;
    int limbs=0;
    std::vector<std::string> tmp_eff(0);
    std::vector<KDL::Frame> tmp_offset(0);
    for(auto& par : params)
    {
        if(par.first.compare("limb")==0)
        {
            std::vector<std::string> strs;
            boost::split(strs, par.second, boost::is_any_of(";"));
            if(strs.size()==1)
            {
                tmp_eff.push_back(strs[0]);
                tmp_offset.push_back(KDL::Frame::Identity());
                limbs++;
            }
            else if(strs.size()==2)
            {
                tmp_eff.push_back(strs[0]);
                KDL::Frame tmp;
                if(!ok(getText(strs[1],tmp)))
                {
                    HIGHLIGHT_NAMED(object_name_,"Invalid limb transform found!")
                    return FAILURE;
                }

                tmp_offset.push_back(tmp);
                limbs++;
            }
            else
            {
                HIGHLIGHT_NAMED(object_name_,"Invalid limbs found!")
                return FAILURE;
            }
        }
    }
    if(limbs>1)
    {
        scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
        return SUCCESS;
    }
    else
    {
        HIGHLIGHT_NAMED(object_name_,"No limbs found!")
        return FAILURE;
    }
  }

  EReturn TaskMap::initBase(tinyxml2::XMLHandle & handle, Server_ptr & server,
      const Scene_map & scene_ptr)
  {
    //!< Clear flags and kinematic scene pointer
    Object::initBase(handle, server);
    if (!server)
    {
      INDICATE_FAILURE
      ;
      return FAILURE;
    }
    server_ = server;
    scene_ = Scene_ptr();  //!< Null pointer

    if (handle.FirstChildElement("Scene").ToElement())
    {
      LOCK(scene_lock_);  //!< Local lock
      const char * name =
          handle.FirstChildElement("Scene").ToElement()->Attribute("name");
      if (name == nullptr)
      {
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }
      auto it = scene_ptr.find(name);
      if (it == scene_ptr.end())
      {
        INDICATE_FAILURE
        ;
        return PAR_ERR;
      }
      scene_ = it->second;
    }
    else
    {
      ERROR("No scene was specified!");
      return PAR_ERR;
    }

    std::vector<std::string> tmp_eff(0);
    std::vector<KDL::Frame> tmp_offset(0);

    tinyxml2::XMLHandle segment_handle(
        handle.FirstChildElement("EndEffector").FirstChildElement("limb"));
    while (segment_handle.ToElement())
    {
      if (!segment_handle.ToElement()->Attribute("segment"))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      tmp_eff.push_back(segment_handle.ToElement()->Attribute("segment"));
      KDL::Frame temp_frame = KDL::Frame::Identity(); //!< Initialise to identity
      if (segment_handle.FirstChildElement("vector").ToElement())
      {
        Eigen::VectorXd temp_vector;
        if (!ok(
            getVector(*(segment_handle.FirstChildElement("vector").ToElement()),
                temp_vector)))
        {
          INDICATE_FAILURE
          return FAILURE;
        }
        if (temp_vector.size() != 3)
        {
          return FAILURE;
        }
        temp_frame.p.x(temp_vector(0));
        temp_frame.p.y(temp_vector(1));
        temp_frame.p.z(temp_vector(2));
      }
      if (segment_handle.FirstChildElement("quaternion").ToElement())
      {
        Eigen::VectorXd temp_vector;
        if (!ok(
            getVector(
                *(segment_handle.FirstChildElement("quaternion").ToElement()),
                temp_vector)))
        {
          INDICATE_FAILURE
          return FAILURE;
        }
        if (temp_vector.size() != 4)
        {
          INDICATE_FAILURE
          return FAILURE;
        }
        temp_frame.M = KDL::Rotation::Quaternion(temp_vector(1), temp_vector(2),
            temp_vector(3), temp_vector(0));
      }
      tmp_offset.push_back(temp_frame);
      segment_handle = segment_handle.NextSiblingElement("limb");
    }

    scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
    if (ok(initDerived(handle)))
    {
      return SUCCESS;
    }
    else
    {
      ERROR("Failed to initialise task '"<<getObjectName() <<"'");
      return FAILURE;
    }
  }

  bool TaskMap::isRegistered(int t)
  {
    if (phi_.size() == 1)
    {
      return true;
    }
    if (phiFlag_(t) == 1)
    {
      if (phiCnt_ != phiFlag_.size())
      {
        WARNING(
            "Task map '"<<object_name_<<"' is hasn't got phi registered at all time steps! phiCnt "<<phiCnt_<<" size "<<phiFlag_.size());
      }
      if (updateJacobian_)
      {
        if (jacFlag_(t) == 1)
        {
          if (jacCnt_ != jacFlag_.size())
          {
            WARNING(
                "Task map '"<<object_name_<<"' is hasn't got phi registered at all time steps! phiCnt "<<phiCnt_<<" size "<<phiFlag_.size());
          }
          return true;
        }
        else
        {
          INDICATE_FAILURE
          ;
          return false;
        }
      }
      else
      {
        return true;
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return false;
    }
  }

  EReturn TaskMap::setTimeSteps(const int T)
  {
    int dim;
    taskSpaceDim(dim);
    phi0_.resize(dim);
    jac0_.resize(dim, scene_->getNumJoints());
    phi_.assign(T, Eigen::VectorXdRef_ptr(phi0_.segment(0, dim)));
    jac_.assign(T,
        Eigen::MatrixXdRef_ptr(jac0_.block(0, 0, dim, scene_->getNumJoints())));
    phiFlag_.resize(T);
    phiFlag_.setZero();
    jacFlag_.resize(T);
    jacFlag_.setZero();
    phiCnt_ = 0;
    jacCnt_ = 0;
    return SUCCESS;
  }

  EReturn TaskMap::registerPhi(Eigen::VectorXdRef_ptr y, int t)
  {
    LOCK(map_lock_);
    phi_.at(t) = y;
    if (phiFlag_(t) == 0)
    {
      phiFlag_(t) = 1;
      phiCnt_++;
    }
    return SUCCESS;
  }

  EReturn TaskMap::registerJacobian(Eigen::MatrixXdRef_ptr J, int t)
  {
    LOCK(map_lock_);
    jac_.at(t) = J;
    if (jacFlag_(t) == 0)
    {
      jacFlag_(t) = 1;
      jacCnt_++;
    }
    return SUCCESS;
  }

  bool TaskMap::getEffReferences()
  {

    if (ok(scene_->getForwardMap(object_name_, eff_phi_)))
    {
      if (updateJacobian_)
      {
        if (ok(scene_->getJacobian(object_name_, eff_jac_)))
        {
          return true;
        }
        else
        {
          INDICATE_FAILURE
          ;
          return false;
        }
      }
      else
      {
        return true;
      }
    }
    else
    {
      INDICATE_FAILURE
      ;
      return false;
    }
  }
}
