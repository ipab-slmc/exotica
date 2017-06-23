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
#include <typeinfo>
#include <Eigen/Dense>
#include <map>
#include <boost/any.hpp>
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
  class Server : public Uncopyable
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
        if (params_.find(name) == params_.end())
          params_[name] = ptr;
        else
          params_.at(name) = ptr;
      }

      /*
       * \brief	List all parameters
       */
      void listParameters();

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
      void getModel(std::string path, robot_model::RobotModelPtr& model, std::string urdf="", std::string srdf="");

      /*
       * \brief	Get robot model
       * @param	path	Robot model name
       * @return	robot model
       */
      robot_model::RobotModelConstPtr getModel(std::string path, std::string urdf="", std::string srdf="");

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
      robot_model::RobotModelPtr loadModel(std::string name, std::string urdf="", std::string srdf="");

      template<typename T>
      void paramCallback(const boost::shared_ptr<T const> & ptr,
          boost::any & param)
      {
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

      /// \brief Robot model cache
      std::map<std::string, robot_model::RobotModelPtr> robot_models_;
  };

  typedef boost::shared_ptr<Server> Server_ptr;
}

#endif /* EXOTICA_INCLUDE_EXOTICA_SERVER_H_ */
