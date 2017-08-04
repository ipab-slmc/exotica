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

exotica::Server_ptr exotica::Server::singleton_server_ = nullptr;
namespace exotica
{
  Server::Server()
      : nh_("/EXOTicaServer"), name_("EXOTicaServer"), sp_(
          2)
  {
    //TODO
    sp_.start();
  }

  Server::~Server()
  {
    sp_.stop();
  }

  void Server::destroy()
  {
      exotica::Server::singleton_server_.reset();
  }

  robot_model::RobotModelPtr Server::loadModel(std::string name, std::string urdf, std::string srdf)
  {
      robot_model::RobotModelPtr model;
      if (hasParam("RobotDescription"))
      {
          EParam<std_msgs::String> robot_description_param;
          getParam("RobotDescription", robot_description_param);
          ROS_INFO_STREAM("Using robot_description at " << robot_description_param->data);
          model = robot_model_loader::RobotModelLoader(robot_description_param->data,false).getModel();
      }
      else if (hasParam(getName() + "/RobotDescription"))
      {
          EParam<std_msgs::String> robot_description_param;
          getParam(getName() + "/RobotDescription", robot_description_param);
          ROS_INFO_STREAM("Using robot_description at " << robot_description_param->data);
          model = robot_model_loader::RobotModelLoader(robot_description_param->data,false).getModel();
      }
      else if (urdf=="" || srdf=="")
      {
          model = robot_model_loader::RobotModelLoader(name,false).getModel();
      }
      else
      {
          model = robot_model_loader::RobotModelLoader(robot_model_loader::RobotModelLoader::Options(urdf,srdf)).getModel();
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

  bool Server::hasParam(const std::string & name)
  {
    if (params_.find(name) == params_.end()) return false;
    return true;
  }

  void Server::listParameters()
  {
    HIGHLIGHT("************* Parameters *************");
    for (auto & it : params_)
    {
      HIGHLIGHT("Parameter: "<<it.first);
    }
    HIGHLIGHT("**************************************");
  }
}
