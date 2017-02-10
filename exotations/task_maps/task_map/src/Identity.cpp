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

#include "Identity.h"

REGISTER_TASKMAP_TYPE("Identity", exotica::Identity);

namespace exotica
{
  Identity::Identity()
      : useRef(false)
  {

  }

  int Identity::getJointID(std::string& name)
  {
    for (int j = 0; j < scene_->getSolver().getJointNames().size(); j++)
    {
      if (scene_->getSolver().getJointNames()[j].compare(name) == 0)
      {
        return j;
      }
    }
    return -1;
  }

  int Identity::getJointIDexternal(std::string& name)
  {
    for (int j = 0; j < posesJointNames->size(); j++)
    {
      if ((*posesJointNames)[j].compare(name) == 0)
      {
        return j;
      }
    }
    return -1;
  }

  void Identity::initialise(std::string& postureName,
      std::vector<std::string>& joints, bool skipUnknown)
  {
    if (!poses || !posesJointNames)
    {
      throw_named("Poses have not been set!");
    }
    std::map<std::string, Eigen::VectorXd>::const_iterator pose = poses->find(
        postureName);
    if (pose != poses->end())
    {
      useRef = true;
      jointMap.resize(0);
      jointRef.resize(0);
      for (int i = 0; i < joints.size(); i++)
      {
        int idext = getJointIDexternal(joints[i]);
        if (idext >= 0)
        {
          int id = getJointID(joints[i]);
          if (id >= 0)
          {
            jointMap.push_back(id);
            jointRef.conservativeResize(jointRef.rows() + 1);
            jointRef(jointRef.rows() - 1) = pose->second(idext);
            continue;
          }
        }
        else
        {
          if (!skipUnknown)
          {
            throw_named("Requesting unknown joint '"<<joints[i]<<"'");
          }
        }
      }
      if (jointMap.size() == 0)
      {
        HIGHLIGHT("No joints have been specified!");
      }
    }
    else
    {
      throw_named("Can't find pose '"<<postureName<<"'");
    }
  }

  void Identity::initialise(const rapidjson::Value& a)
  {
    if (poses && posesJointNames)
    {
        std::string postureName;
        getJSON(a["postureName"], postureName);
        std::vector<std::string> joints;
        getJSON(a["joints"], joints);
        initialise(postureName, joints);
    }
    else
    {
      throw_named("Poses have not been set!");
    }
  }

  void Identity::update(const Eigen::Ref<const Eigen::VectorXd> x, const int t)
  {
    if (!isRegistered(t))
    {
      throw_named("Not fully initialized!");
    }
//        std::cout<<"Updating ";
//        for(int i=0;i<jointMap.size();i++)
//        	std::cout<<(*posesJointNames)[jointMap[i]]<<" ";
//        std::cout<<std::endl;
    if (x.rows() >= PHI.rows())
    {
      if (useRef)
      {
        if (updateJacobian_)
        JAC.setZero();
        for (int i = 0; i < jointMap.size(); i++)
        {
          PHI(i) = x(jointMap[i]) - jointRef(i);
          if (updateJacobian_)
          {
            JAC(i, jointMap[i]) = 1.0;
          }
        }
      }
      else
      {
        PHI = x;
        if (updateJacobian_)
        {
          JAC = Eigen::MatrixXd::Identity(x.rows(), x.rows());
        }
      }
    }
    else
    {
      throw_named("Size mismatch "<<x.rows()<<"!="<<PHI.rows());
    }
  }

  void Identity::Instantiate(IdentityInitializer& init)
  {
      if(init.JointRef.rows()>0)
      {
          jointRef = init.JointRef;
          if(init.JointMap.size()>0)
          {
              jointMap = init.JointMap;
              if(jointMap.size()!=jointRef.rows()) throw_named("Incorrect joint map size!");
          }
          else
          {
              jointMap.resize(jointRef.rows());
              for (int i = 0; i < jointRef.rows(); i++)
                  jointMap[i] = i;
          }
          useRef = true;
      }
  }

  void Identity::initDerived(tinyxml2::XMLHandle & handle)
  {
    // Load the goal
    if (handle.FirstChildElement("Ref").ToElement())
    {
        getVector(*(handle.FirstChildElement("Ref").ToElement()), jointRef);
        jointMap.resize(jointRef.rows());
        for (int i = 0; i < jointRef.rows(); i++)
            jointMap[i] = i;
        useRef = true;
    }
  }

  void Identity::taskSpaceDim(int & task_dim)
  {
    if (useRef)
    {
      task_dim = jointMap.size();
    }
    else
    {
      task_dim = scene_->getNumJoints();
    }
  }
}
