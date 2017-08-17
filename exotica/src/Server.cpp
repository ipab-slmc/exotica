/*
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

#include "exotica/Server.h"
#include <fstream>

exotica::Server_ptr exotica::Server::singleton_server_ = nullptr;
namespace exotica
{

  RosNode::RosNode(std::shared_ptr<ros::NodeHandle> nh, int numThreads) : nh_(nh), sp_(numThreads)
  {
      sp_.start();
  }

  RosNode::~RosNode()
  {
      sp_.stop();
  }

  Server::Server() : name_("EXOTicaServer"), node_(nullptr)
  {

  }

  Server::~Server()
  {

  }

  void Server::destroy()
  {
      exotica::Server::singleton_server_.reset();
  }

  inline bool exists(const std::string& path)
  {
    std::ifstream file(path.c_str());
    return (bool)file;
  }

  robot_model::RobotModelPtr loadModelImpl(const std::string& urdf, const std::string & srdf)
  {
      rdf_loader::RDFLoader loader(urdf, srdf);
      const boost::shared_ptr<srdf::Model>& srdf_ = loader.getSRDF() ? loader.getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
      if(loader.getURDF())
      {
          return robot_model::RobotModelPtr(new robot_model::RobotModel(loader.getURDF(), srdf_));
      }
      else
      {
          throw_pretty("Can't load robot model from URDF!");
      }
  }

  robot_model::RobotModelPtr Server::loadModel(std::string name, std::string urdf, std::string srdf)
  {
      robot_model::RobotModelPtr model;
      if (hasParam("RobotDescription"))
      {
          std::string robot_description_param;
          getParam("RobotDescription", robot_description_param);
          ROS_INFO_STREAM("Using robot_description at " << robot_description_param);
          model = robot_model_loader::RobotModelLoader(robot_description_param,false).getModel();
      }
      else if (hasParam(getName() + "/RobotDescription"))
      {
          std::string robot_description_param;
          getParam(getName() + "/RobotDescription", robot_description_param);
          ROS_INFO_STREAM("Using robot_description at " << robot_description_param);
          model = robot_model_loader::RobotModelLoader(robot_description_param,false).getModel();
      }
      else if(urdf=="" || srdf=="")
      {
          model = robot_model_loader::RobotModelLoader(name,false).getModel();
      }
      else if(exists(urdf) && exists(srdf))
      {
          std::string urdf_, srdf_;
          {
              std::ifstream t(urdf);
              urdf_ = std::string((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
          }
          {
              std::ifstream t(srdf);
              srdf_ = std::string((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
          }
          model = loadModelImpl(urdf_,srdf_);
      }
      else
      {
          model = loadModelImpl(urdf,srdf);
      }

      if (model)
      {
        robot_models_[name] = model;
      }
      else
      {
        throw_pretty("Couldn't load the model at path " << name << "!");
      }
      return model;
  }

  void Server::getModel(std::string path, robot_model::RobotModelPtr & model, std::string urdf, std::string srdf)
  {
    if (robot_models_.find(path) != robot_models_.end())
    {
      model = robot_models_[path];
    }
    else
    {
      model = loadModel(path,urdf,srdf);
    }
  }

  robot_model::RobotModelConstPtr Server::getModel(std::string path, std::string urdf, std::string srdf)
  {
    if (robot_models_.find(path) != robot_models_.end())
    {
      return robot_models_[path];
    }
    else
    {
      return loadModel(path,urdf,srdf);
    }
  }

  bool Server::hasModel(const std::string & path)
  {
    return robot_models_.find(path) != robot_models_.end();
  }

  std::string Server::getName()
  {
    return name_;
  }

}
