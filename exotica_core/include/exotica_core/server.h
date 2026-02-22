//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_SERVER_H_
#define EXOTICA_CORE_SERVER_H_

#include <map>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <exotica_core/tools/exception.h>
#include <exotica_core/tools/uncopyable.h>

namespace exotica
{
// Implementation of EXOTica Server class
class Server : public Uncopyable
{
public:
    /// \brief Get the server
    static std::shared_ptr<Server> Instance()
    {
        if (!singleton_server_) singleton_server_.reset(new Server);
        return singleton_server_;
    }
    virtual ~Server();

    /// \brief Check if a robot model exist
    /// @param path Robot model name
    /// @return True if exist, false otherwise
    bool HasModel(const std::string &path);

    /// \brief Get robot model
    /// @param path Robot model name
    /// @param model Robot model
    void GetModel(const std::string &path, moveit::core::RobotModelPtr &model, const std::string &urdf = "", const std::string &srdf = "");

    /// \brief Get robot model
    /// @param path Robot model name
    /// @return robot model
    moveit::core::RobotModelConstPtr GetModel(const std::string &path, const std::string &urdf = "", const std::string &srdf = "");

    /// \brief Get the name of the server
    /// @return Server name
    std::string GetName();

    inline static void InitRos(rclcpp::Node::SharedPtr node)
    {
        Instance()->nh_ = node;
        Instance()->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        Instance()->executor_->add_node(node);
        Instance()->spinner_thread_ = std::thread([&]() { Instance()->executor_->spin(); });
        Instance()->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }

    inline static bool IsRos() { return Instance()->nh_ != nullptr; }

    inline static bool IsOk() { return IsRos() ? rclcpp::ok() : true; }

    inline static rclcpp::Node::SharedPtr GetNode()
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        return Instance()->nh_;
    }

    template <typename T>
    static bool GetParam(const std::string &name, T &param)
    {
        if (!IsRos()) return false;
        auto node = GetNode();
        if (!node->has_parameter(name))
        {
            try
            {
                node->declare_parameter<T>(name);
            }
            catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
            {
            }
        }
        rclcpp::Parameter p;
        bool ret = node->get_parameter(name, p);
        if (ret) param = p.get_value<T>();
        return ret;
    }

    template <typename T>
    static void SetParam(const std::string &name, T &param)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        auto node = GetNode();
        if (!node->has_parameter(name))
        {
            try
            {
                node->declare_parameter<T>(name, param);
            }
            catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
            {
            }
        }
        node->set_parameter(rclcpp::Parameter(name, param));
    }

    inline bool static HasParam(const std::string &name)
    {
        if (IsRos())
        {
            return GetNode()->has_parameter(name);
        }
        else
        {
            return false;
        }
    }

    template <typename T>
    static typename rclcpp::Publisher<T>::SharedPtr Advertise(const std::string &topic, int depth = 1, bool latch = false)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        rclcpp::QoS qos(depth);
        if (latch) qos.transient_local();
        return GetNode()->create_publisher<T>(topic, qos);
    }

    template <typename T, typename CallbackT>
    static typename rclcpp::Subscription<T>::SharedPtr Subscribe(const std::string &topic, CallbackT &&callback, int depth = 1)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        return GetNode()->create_subscription<T>(topic, depth, std::forward<CallbackT>(callback));
    }

    static void SendTransform(const geometry_msgs::msg::TransformStamped &transform)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        Instance()->tf_broadcaster_->sendTransform(transform);
    }

    static void SendTransform(const std::vector<geometry_msgs::msg::TransformStamped> &transforms)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        Instance()->tf_broadcaster_->sendTransform(transforms);
    }

    static void Destroy();

private:
    Server();
    static std::shared_ptr<Server> singleton_server_;
    ///	\brief	Make sure the singleton does not get copied
    Server(Server const &) = delete;
    void operator=(Server const &) = delete;
    moveit::core::RobotModelPtr LoadModel(const std::string &name, const std::string &urdf = "", const std::string &srdf = "");

    /// \brief	The name of this server
    std::string name_;

    rclcpp::Node::SharedPtr nh_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    std::thread spinner_thread_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// \brief Robot model cache
    std::map<std::string, moveit::core::RobotModelPtr> robot_models_;
};

typedef std::shared_ptr<Server> ServerPtr;
}  // namespace exotica

#endif  // EXOTICA_CORE_SERVER_H_
