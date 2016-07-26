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

#ifndef EXOTICA_INCLUDE_EXOTICA_SERVER_H_
#define EXOTICA_INCLUDE_EXOTICA_SERVER_H_

#include <ros/ros.h>
#include "Tools.h"
#include "tinyxml2/tinyxml2.h"
#include <typeinfo>
#include <Eigen/Dense>
#include <map>
#include <boost/any.hpp>
#include <boost/thread/mutex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <exotica/Tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
namespace exotica
{
  //	EXOTica Parameter type
  template<typename T>
  using EParam = boost::shared_ptr<T>;

  //	Implementation of EXOTica Server class
  class Server
  {

    public:
      /*
       * \brief	Get the server
       */
      static boost::shared_ptr<Server> Instance()
      {
        if (!singleton_server_) singleton_server_.reset(new Server);
        return singleton_server_;
      }
      virtual ~Server();

      void initialise(tinyxml2::XMLHandle & handle);

      /*
       * \brief	Create a new parameter entry from an XML handle
       * @param	ns				namespace
       * @param	tmp_handle		XML handle
       */
      void createParam(const std::string & ns,
          tinyxml2::XMLHandle & tmp_handle);

      /*
       * \brief	Check if a parameter is exist
       * @param	name		Parameter name
       * @return	True if exist, false otherwise
       */
      bool hasParam(const std::string & name);

      /*
       * \brief	Get the latest available parameter
       * @param	name		Parameter name
       * @param	ptr			Parameter pointer
       */
      template<typename T>
      void getParam(const std::string & name, EParam<T> & ptr)
      {
        LOCK(param_lock_);
        if (params_.find(name) == params_.end())
        {
          WARNING_NAMED(name_,"Param " << name << " does not exist");
          listParameters();
          throw_pretty("Can't find parameter '"<<name<<"'");
        }
        ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
      }

      /*
       * \brief	Assign new value to the parameter
       * @param	name		Parameter name
       * @param	ptr			Pointer to the parameter
       */
      template<typename T>
      void setParam(const std::string & name, const EParam<T> & ptr)
      {
        LOCK(param_lock_);
        if (params_.find(name) == params_.end())
          params_[name] = ptr;
        else
          params_.at(name) = ptr;
      }

      /*
       * \brief	List all parameters
       */
      void listParameters();

      /*
       * \brief	Register a new parameter
       * @param	ns		Namespace
       * @param	handle	xml handle
       * @param	ptr		output pointer
       */
      template<typename T>
      void registerParam(const std::string & ns,
          tinyxml2::XMLHandle & handle, EParam<T> & ptr)
      {
        if (!handle.ToElement())
        {
          WARNING_NAMED(ns,
              "Register parameter failed, XML tag does not exist, check the XML file");
          ptr = boost::shared_ptr<T>(new T());
          return;
        }
        std::string name;
        if (handle.ToElement()->Attribute("source"))
        {
          name = handle.ToElement()->Attribute("source");
          if (params_.find(name) != params_.end())
          {
            ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
          }
          else
          {
            throw_pretty("Can't find source '"<<name<<"'");
          }
        }

        //	Check if it is an existing ROS parameter
        name = ns + "/" + handle.ToElement()->Name();
        if (params_.find(name) != params_.end())
        {
          ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
        }

        //	If not, create a new parameter entry
        if (handle.ToElement()->Attribute("topic"))
        {
          std::string topic = handle.ToElement()->Attribute("topic");
          //	If a topic is specified
          params_[name] = boost::shared_ptr<T>(new T);
          subs_[name] = nh_.subscribe<T>(topic, 1,
              boost::bind(&exotica::Server::paramCallback<T>, this, _1,
                  params_.at(name)));
        }
        else if (handle.ToElement()->Attribute("rosparam"))
        {
          std::string rosparam = handle.ToElement()->Attribute("rosparam");
          if (typeid(T) == typeid(std::string))
          {
            std::string val;
            nh_.getParam(rosparam, val);
            params_[name] = boost::shared_ptr<std::string>(
                new std::string(val));
          }
          else if (typeid(T) == typeid(double))
          {
            double val;
            nh_.getParam(rosparam, val);
            params_[name] = boost::shared_ptr<double>(new double(val));
          }
          else if (typeid(T) == typeid(int))
          {
            int val;
            nh_.getParam(rosparam, val);
            params_[name] = boost::shared_ptr<int>(new int(val));
          }
          else
          {
            throw_pretty("ROS parameter " << name << " [" << typeid(T).name()
                << "] is not supported\n");
          }
        }
        else
        {
          //	If not topic is specified, it should just contain a static value
          if (typeid(T) == typeid(std_msgs::Bool))
          {
            std_msgs::Bool tmp;
            const char* tmp_str = handle.ToElement()->GetText();
            if (strcmp(tmp_str, "true") == 0 || strcmp(tmp_str, "1") == 0)
              tmp.data = true;
            else
              tmp.data = false;
            params_[name] = boost::shared_ptr<std_msgs::Bool>(
                new std_msgs::Bool(tmp));
          }
          else if (typeid(T) == typeid(std_msgs::Int64))
          {
            double tmp;
            getDouble(*handle.ToElement(), tmp);
            std_msgs::Int64 val;
            val.data = (int) tmp;
            params_[name] = boost::shared_ptr<std_msgs::Int64>(
                new std_msgs::Int64(val));
          }
          else if (typeid(T) == typeid(std_msgs::Float64))
          {
            double tmp;
            getDouble(*handle.ToElement(), tmp);
            std_msgs::Float64 val;
            val.data = tmp;
            params_[name] = boost::shared_ptr<std_msgs::Float64>(
                new std_msgs::Float64(val));
          }
          else if (typeid(T) == typeid(exotica::Vector))
          {
            exotica::Vector vec;
            getStdVector(*handle.ToElement(), vec.data);
            params_[name] = boost::shared_ptr<exotica::Vector>(
                new exotica::Vector(vec));
          }
          else if (typeid(T) == typeid(std_msgs::Bool))
          {
            std_msgs::Bool val;
            bool b;
            getBool(*handle.ToElement(), b);
            val.data = b;
            params_[name] = boost::shared_ptr<std_msgs::Bool>(
                new std_msgs::Bool(val));
          }
          else if (typeid(T) == typeid(std_msgs::String))
          {
            std_msgs::String val;
            const char * atr = handle.ToElement()->GetText();
            if (!atr)
            {
              throw_pretty("Parsing error");
            }
            val.data = std::string(atr);
            params_[name] = boost::shared_ptr<std_msgs::String>(
                new std_msgs::String(val));
          }
          else
          {
            throw_pretty("Parameter " << name << " [" << typeid(T).name()
                << "] is not supported\n");
          }
        }
        ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
      }

      /*
       * \brief	Register a parameter to ROS parameter
       * @param	name	Parameter name
       * @param	topic	ROS parameter nametf::TransformListener listener;
       * @param	ptr		Parameter pointer
       */
      template<typename T>
      void registerRosParam(const std::string & ns,
          tinyxml2::XMLHandle & handle, EParam<T> & ptr)
      {
        if (!handle.ToElement())
        {
          throw_pretty(ns<<" register parameter failed, check the XML file");
        }
        std::string name;
        if (handle.ToElement()->Attribute("source"))
        {
          name = handle.ToElement()->Attribute("source");
          if (params_.find(name) != params_.end())
          {
            ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
          }
          else
          {
            throw_pretty("Can't find attribute '"<<name<<"'");
          }
        }

        //	Check if it is an existing ROS parameter
        name = ns + "/" + handle.ToElement()->Name();
        if (params_.find(name) != params_.end())
        {
          ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
        }

        //	If not, create a new parameter entry
        if (handle.ToElement()->Attribute("rosparam"))
        {
          std::string rosparam = handle.ToElement()->Attribute("rosparam");
          if (typeid(T) == typeid(std::string))
          {
            std::string val;
            nh_.getParam(rosparam, val);
            params_[name] = boost::shared_ptr<std::string>(
                new std::string(val));
          }
          else if (typeid(T) == typeid(double))
          {
            double val;
            nh_.getParam(rosparam, val);
            params_[name] = boost::shared_ptr<double>(new double(val));
          }
          else if (typeid(T) == typeid(int))
          {
            int val;
            nh_.getParam(rosparam, val);
            params_[name] = boost::shared_ptr<int>(new int(val));
          }
          else
          {
            throw_pretty("ROS parameter " << name << " [" << typeid(T).name()
                << "] is not supported");
          }
        }
        ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
      }

      /*
       * \brief	Register a parameter to ROS topic
       * @param	name	Parameter name
       * @param	topic	ROS topic
       * @param	ptr		Parameter pointer
       */
      template<typename T>
      void registerParam(const std::string & name, const std::string & topic,
          EParam<T> & ptr)
      {
        if (name="")
        {
          throw_pretty("Invalid parameter name!");
        }

        //	If the parameter exists, return the pointer
        if (params_.find(name) != params_.end())
        {
          ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
          return;
        }

        if (topic=="")
        {
          params_[name] = boost::shared_ptr<T>(new T);
          subs_[name] = nh_.subscribe<T>(topic, 1,
              boost::bind(&exotica::Server::paramCallback<T>, this, _1,
                  params_.at(name)));
        }
        else
        {
          throw_pretty("Ivalid topic name!");
        }

        ptr = boost::any_cast<boost::shared_ptr<T>>(params_.at(name));
      }

      /*
       * \brief	Register static parameters, since we need to read a particular type from xml,
       * 			we can not use template. can we?
       * @param	name	Parameter name
       * @param	handle	XML handle
       */
      void registerStatic(const std::string & name,
          tinyxml2::XMLHandle & handle, EParam<int> & ptr)
      {
        double tmp;
        getDouble(*handle.ToElement(), tmp);
        int val = tmp;
        *boost::any_cast<boost::shared_ptr<int>>(params_.at(name)) = val;
      }

      template<typename T>
      ros::Publisher advertise(const std::string &topic, uint32_t queue_size,
          bool latch = false)
      {
        return nh_.advertise<T>(topic, queue_size, latch);
      }

      /*
       * \brief	Check if a robot model exist
       * @param	path	Robot model name
       * @return	True if exist, false otherwise
       */
      bool hasModel(const std::string & path);

      /*
       * \brief	Get robot model
       * @param	path	Robot model name
       * @param	model	Robot model
       */
      void getModel(std::string path, robot_model::RobotModelPtr& model);

      /*
       * \brief	Get robot model
       * @param	path	Robot model name
       * @return	robot model
       */
      robot_model::RobotModelConstPtr getModel(std::string path);

      /*
       * \brief	Get the name of ther server
       * @return	Server name
       */
      std::string getName();
    private:
      /*
       * \brief	Constructor
       */
      Server();
      static boost::shared_ptr<Server> singleton_server_;
      ///	\brief	Make sure the singleton does not get copied
      Server(Server const&) = delete;
      void operator=(Server const&) = delete;

      template<typename T>
      void paramCallback(const boost::shared_ptr<T const> & ptr,
          boost::any & param)
      {
        LOCK(param_lock_);
        *boost::any_cast<boost::shared_ptr<T>>(param) = *ptr;
      }

      /// \brief	The name of this server
      std::string name_;

      /// \brief	ROS node handle
      ros::NodeHandle nh_;

      ///	\brief	spinner
      ros::AsyncSpinner sp_;

      std::map<std::string, ros::Subscriber> subs_;

      /// \brief	Parameters map <name, parameter pointer>
      std::map<std::string, boost::any> params_;

      /// \brief	<param_name, param_topic>
      std::map<std::string, std::string> topics_;

      /// \brief	Mutex locker
      boost::mutex param_lock_;

      /// \brief Robot model cache
      std::map<std::string, robot_model::RobotModelPtr> robot_models_;
  };

  typedef boost::shared_ptr<Server> Server_ptr;
}

#endif /* EXOTICA_INCLUDE_EXOTICA_SERVER_H_ */
