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

  void TaskMap::debug()
  {
    //  You need to implement this in your own taskmap
  }

  void TaskMap::initialise(const rapidjson::Value& a)
  {
    throw_named("This has to be implemented in the derived class!");
  }

  void TaskMap::initialise(const rapidjson::Value& a, Server_ptr & server,
      const Scene_ptr & scene_ptr, PlanningProblem_ptr prob)
  {
    getJSON(a["class"], object_name_);
      if (!server)
      {
        throw_named("Invalid server!");
      }
      std::vector<std::pair<std::string, std::string> > tmp;
      initialiseManual(object_name_, server, scene_ptr, prob, tmp);
      initialise(a);
  }

  void TaskMap::initialiseManual(std::string name, Server_ptr & server,
      const Scene_ptr & scene_ptr, boost::shared_ptr<PlanningProblem> prob,
      std::vector<std::pair<std::string, std::string> >& params)
  {
    object_name_ = name + std::to_string((unsigned long) this);
    server_ = server;
    scene_ = scene_ptr;
    poses = prob->poses;
    posesJointNames = prob->posesJointNames;
    int limbs = 0;
    std::vector<std::string> tmp_eff(0);
    std::vector<KDL::Frame> tmp_offset(0);
    for (auto& par : params)
    {
      if (par.first.compare("limb") == 0)
      {
        std::vector<std::string> strs;
        boost::split(strs, par.second, boost::is_any_of(";"));
        if (strs.size() == 1)
        {
          tmp_eff.push_back(strs[0]);
          tmp_offset.push_back(KDL::Frame::Identity());
          limbs++;
        }
        else if (strs.size() == 2)
        {
          tmp_eff.push_back(strs[0]);
          KDL::Frame tmp;
          try
          {
            getText(strs[1], tmp);
          }
          catch (Exception e)
          {
            throw_named("Invalid limb transform found!");
          }

          tmp_offset.push_back(tmp);
          limbs++;
        }
        else
        {
          throw_named("Invalid limbs found!")
        }
      }
    }
    if (limbs > 1)
    {
      scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
    }
    else
    {
      HIGHLIGHT("No limbs found!");
    }
  }

  void TaskMap::initBase(tinyxml2::XMLHandle & handle, Server_ptr & server,
      const Scene_ptr & scene_ptr)
  {
    //!< Clear flags and kinematic scene pointer
    Object::initBase(handle, server);
    if (!server)
    {
      throw_named("Invalid server!");
    }
    server_ = server;
    scene_ = scene_ptr;  //!< Null pointer

//    if (handle.FirstChildElement("Scene").ToElement())
//    {
//      LOCK(scene_lock_);  //!< Local lock
//      const char * name =
//          handle.FirstChildElement("Scene").ToElement()->Attribute("name");
//      if (name == nullptr)
//      {
//        throw_named("Invalid scene name!");
//      }
//      auto it = scene_ptr.find(name);
//      if (it == scene_ptr.end())
//      {
//        throw_named("Can't find the scene!");
//      }
//      scene_ = it->second;
//    }
//    else
//    {
//      throw_named("No scene was specified!");
//    }

    std::vector<std::string> tmp_eff(0);
    std::vector<KDL::Frame> tmp_offset(0);

    tinyxml2::XMLHandle segment_handle(
        handle.FirstChildElement("EndEffector").FirstChildElement("limb"));
    while (segment_handle.ToElement())
    {
      if (!segment_handle.ToElement()->Attribute("segment"))
      {
        throw_named("Invalid segment!");
      }
      tmp_eff.push_back(segment_handle.ToElement()->Attribute("segment"));
      KDL::Frame temp_frame = KDL::Frame::Identity(); //!< Initialise to identity
      if (segment_handle.FirstChildElement("vector").ToElement())
      {
        Eigen::VectorXd temp_vector;
        getVector(*(segment_handle.FirstChildElement("vector").ToElement()),
                temp_vector);
        if (temp_vector.size() != 3)
        {
          throw_named("Invalid vector size!");
        }
        temp_frame.p.x(temp_vector(0));
        temp_frame.p.y(temp_vector(1));
        temp_frame.p.z(temp_vector(2));
      }
      if (segment_handle.FirstChildElement("quaternion").ToElement())
      {
        Eigen::VectorXd temp_vector;
        getVector(
                *(segment_handle.FirstChildElement("quaternion").ToElement()),
                temp_vector);
        if (temp_vector.size() != 4)
        {
          throw_named("Invalid vector size!");
        }
        temp_frame.M = KDL::Rotation::Quaternion(temp_vector(1), temp_vector(2),
            temp_vector(3), temp_vector(0));
      }
      tmp_offset.push_back(temp_frame);
      segment_handle = segment_handle.NextSiblingElement("limb");
    }

    scene_->appendTaskMap(getObjectName(), tmp_eff, tmp_offset);
    initDerived(handle);
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

  void TaskMap::setTimeSteps(const int T)
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
  }

  void TaskMap::registerPhi(Eigen::VectorXdRef_ptr y, int t)
  {
    LOCK(map_lock_);
    phi_.at(t) = y;
    if (phiFlag_(t) == 0)
    {
      phiFlag_(t) = 1;
      phiCnt_++;
    }
  }

  void TaskMap::registerJacobian(Eigen::MatrixXdRef_ptr J, int t)
  {
    LOCK(map_lock_);
    jac_.at(t) = J;
    if (jacFlag_(t) == 0)
    {
      jacFlag_(t) = 1;
      jacCnt_++;
    }
  }

  bool TaskMap::getEffReferences()
  {

      scene_->getForwardMap(object_name_, eff_phi_);
      if (updateJacobian_)
      {
        scene_->getJacobian(object_name_, eff_jac_);
        return true;
      }
      else
      {
        return true;
      }
  }
}
