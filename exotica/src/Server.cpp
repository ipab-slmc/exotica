/*
 * Server.cpp
 *
 *  Created on: 20 Nov 2014
 *      Author: yiming
 */

#include "exotica/Server.h"
exotica::Server_ptr exotica::Server::singleton_server_ = NULL;
namespace exotica
{
  Server::Server()
      : nh_(new ros::NodeHandle("/EXOTicaServer")), name_("EXOTicaServer"), sp_(
          2)
  {
    //TODO
    sp_.start();
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
    if (!singleton_server_)
    {
      name_ = handle.ToElement()->Attribute("name");
      std::string ns = name_;
    }
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
