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
exotica::Server_ptr exotica::Server::singleton_server_ = NULL;
namespace exotica
{
  Server::Server()
      : nh_(new ros::NodeHandle("~")), name_("EXOTicaServer"), sp_(
          2)
  {
  }

  Server::~Server()
  {
    sp_.stop();
  }

  EReturn Server::getModel(std::string path, robot_model::RobotModelPtr & model)
  {
    if (robot_models_.find(path) != robot_models_.end())
    {
      model = robot_models_[path];
      return SUCCESS;
    }
    else
    {
      model = robot_model_loader::RobotModelLoader(path).getModel();
      if (model)
      {
        robot_models_[path] = model;
        return SUCCESS;
      }
      else
      {
        INDICATE_FAILURE
        ;
        return FAILURE;
      }
    }
  }

  robot_model::RobotModelConstPtr Server::getModel(std::string path)
  {
    if (robot_models_.find(path) != robot_models_.end())
    {
      return robot_models_[path];
    }
    else
    {
      return robot_model_loader::RobotModelLoader(path).getModel();
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

  EReturn Server::initialise(tinyxml2::XMLHandle & handle)
  {
    static bool first = true;
    name_ = handle.ToElement()->Attribute("name");
    nh_.reset(new ros::NodeHandle(name_));
    sp_.start();
    tinyxml2::XMLHandle param_handle(handle.FirstChildElement("Parameters"));
    tinyxml2::XMLHandle tmp_handle = param_handle.FirstChild();
    while (tmp_handle.ToElement())
    {
      if (!ok(createParam(name_, tmp_handle)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      tmp_handle = tmp_handle.NextSibling();
    }
    if (first)
    {
      HIGHLIGHT_NAMED(name_, "EXOTica Server Initialised");
      first = false;
    }
    return SUCCESS;
  }

  EReturn Server::createParam(const std::string & ns,
      tinyxml2::XMLHandle & tmp_handle)
  {
    std::string name = ns + "/" + tmp_handle.ToElement()->Name();
    std::string type = tmp_handle.ToElement()->Attribute("type");
    if (type.compare("int") == 0)
    {
      double dou;
      if (!ok(
          getDouble(*tmp_handle.FirstChildElement("default").ToElement(), dou)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      std_msgs::Int64 val;
      val.data = dou;
      params_[name] = boost::shared_ptr<std_msgs::Int64>(
          new std_msgs::Int64(val));
    }
    else if (type.compare("double") == 0)
    {
      double dou;
      if (!ok(
          getDouble(*tmp_handle.FirstChildElement("default").ToElement(), dou)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      std_msgs::Float64 val;
      val.data = dou;
      params_[name] = boost::shared_ptr<std_msgs::Float64>(
          new std_msgs::Float64(val));
    }
    else if (type.compare("vector") == 0)
    {
      exotica::Vector vec;
      if (!ok(
          getStdVector(*tmp_handle.FirstChildElement("default").ToElement(),
              vec.data)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      params_[name] = boost::shared_ptr<exotica::Vector>(
          new exotica::Vector(vec));
    }
    else if (type.compare("bool") == 0)
    {
      bool val;
      if (!ok(
          getBool(*tmp_handle.FirstChildElement("default").ToElement(), val)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      std_msgs::Bool ros_bool;
      ros_bool.data = val;
      params_[name] = boost::shared_ptr<std_msgs::Bool>(
          new std_msgs::Bool(ros_bool));
    }
    else if (type.compare("boollist") == 0)
    {
      exotica::BoolList boollist;
      std::vector<bool> vec;
      if (!ok(
          getBoolVector(*tmp_handle.FirstChildElement("default").ToElement(),
              vec)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      boollist.data.resize(vec.size());
      for (int i = 0; i < vec.size(); i++)
        boollist.data[i] = vec[i];
      params_[name] = boost::shared_ptr<exotica::BoolList>(
          new exotica::BoolList(boollist));
    }
    else if (type.compare("string") == 0)
    {
      std::string str =
          tmp_handle.FirstChildElement("default").ToElement()->GetText();
      if (str.size() == 0)
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      std_msgs::String ros_s;
      ros_s.data =
          tmp_handle.FirstChildElement("default").ToElement()->GetText();
      params_[name] = boost::shared_ptr<std_msgs::String>(
          new std_msgs::String(ros_s));
    }
    else if (type.compare("stringlist") == 0)
    {
      exotica::StringList list;
      if (!ok(
          getList(*tmp_handle.FirstChildElement("default").ToElement(),
              list.strings)))
      {
        INDICATE_FAILURE
        return FAILURE;
      }
      params_[name] = boost::shared_ptr<exotica::StringList>(
          new exotica::StringList(list));
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    INFO("Register new paramter "<<name)
    return SUCCESS;
  }

  bool Server::hasParam(const std::string & name)
  {
    LOCK(param_lock_);
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
