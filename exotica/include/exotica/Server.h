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
#include <Eigen/Dense>
#include <boost/any.hpp>
#include <map>
#include <typeinfo>
#include "Tools.h"

#include <eigen_conversions/eigen_msg.h>
#include <exotica/Tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
namespace exotica
{
class RosNode
{
public:
    RosNode() = delete;
    RosNode(std::shared_ptr<ros::NodeHandle> nh, int numThreads = 2);
    ~RosNode();
    inline ros::NodeHandle &getNodeHandle() { return *nh_; }
    inline tf::TransformBroadcaster &getTF() { return tf_; }
protected:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::AsyncSpinner sp_;
    tf::TransformBroadcaster tf_;
};

//	Implementation of EXOTica Server class
class Server : public Uncopyable
{
public:
    /*
       * \brief	Get the server
       */
    static std::shared_ptr<Server> Instance()
    {
        if (!singleton_server_) singleton_server_.reset(new Server);
        return singleton_server_;
    }
    virtual ~Server();

    /*
       * \brief	Check if a robot model exist
       * @param	path	Robot model name
       * @return	True if exist, false otherwise
       */
    bool hasModel(const std::string &path);

    /*
       * \brief	Get robot model
       * @param	path	Robot model name
       * @param	model	Robot model
       */
    void getModel(std::string path, robot_model::RobotModelPtr &model, std::string urdf = "", std::string srdf = "");

    /*
       * \brief	Get robot model
       * @param	path	Robot model name
       * @return	robot model
       */
    robot_model::RobotModelConstPtr getModel(std::string path, std::string urdf = "", std::string srdf = "");

    /*
       * \brief	Get the name of ther server
       * @return	Server name
       */
    std::string getName();

    inline static void InitRos(std::shared_ptr<ros::NodeHandle> nh, int numThreads = 2)
    {
        Instance()->node_.reset(new RosNode(nh, numThreads));
    }

    inline static bool isRos() { return Instance()->node_ != nullptr; }
    inline static ros::NodeHandle &getNodeHandle()
    {
        if (!isRos()) throw_pretty("EXOTica server not initialized as ROS node!");
        return Instance()->node_->getNodeHandle();
    }

    template <typename T>
    static bool getParam(const std::string &name, T &param)
    {
        return Instance()->getNodeHandle().getParam(name, param);
    }

    template <typename T>
    static void setParam(const std::string &name, T &param)
    {
        Instance()->getNodeHandle().setParam(name, param);
    }

    inline bool static hasParam(const std::string &name)
    {
        if (isRos())
        {
            return Instance()->getNodeHandle().hasParam(name);
        }
        else
        {
            return false;
        }
    }

    template <typename T, typename... Args>
    static ros::Publisher advertise(Args &&... args)
    {
        return Instance()->getNodeHandle().advertise<T>(std::forward<Args>(args)...);
    }

    template <typename T, typename... Args>
    static ros::Subscriber subscribe(Args &&... args)
    {
        return Instance()->getNodeHandle().subscribe<T>(std::forward<Args>(args)...);
    }

    template <typename T>
    static ros::ServiceClient serviceClient(const std::string &service_name, bool persistent = false)
    {
        return Instance()->getNodeHandle().serviceClient<T>(service_name, persistent);
    }

    static void sendTransform(const tf::StampedTransform &transform)
    {
        if (!isRos()) throw_pretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->getTF().sendTransform(transform);
    }

    static void sendTransform(const std::vector<tf::StampedTransform> &transforms)
    {
        if (!isRos()) throw_pretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->getTF().sendTransform(transforms);
    }

    static void sendTransform(const geometry_msgs::TransformStamped &transform)
    {
        if (!isRos()) throw_pretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->getTF().sendTransform(transform);
    }

    static void sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms)
    {
        if (!isRos()) throw_pretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->getTF().sendTransform(transforms);
    }

    static void destroy();

private:
    /*
       * \brief	Constructor
       */
    Server();
    static std::shared_ptr<Server> singleton_server_;
    ///	\brief	Make sure the singleton does not get copied
    Server(Server const &) = delete;
    void operator=(Server const &) = delete;
    robot_model::RobotModelPtr loadModel(std::string name, std::string urdf = "", std::string srdf = "");

    /// \brief	The name of this server
    std::string name_;

    std::shared_ptr<RosNode> node_;

    /// \brief Robot model cache
    std::map<std::string, robot_model::RobotModelPtr> robot_models_;
};

typedef std::shared_ptr<Server> Server_ptr;
}

#endif /* EXOTICA_INCLUDE_EXOTICA_SERVER_H_ */
