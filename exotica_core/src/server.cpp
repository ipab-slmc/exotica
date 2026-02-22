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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>

#include <exotica_core/server.h>
#include <exotica_core/tools.h>

#include <typeinfo>
#include <boost/any.hpp>
#include <rclcpp/rclcpp.hpp>

exotica::ServerPtr exotica::Server::singleton_server_ = nullptr;
namespace exotica
{
Server::Server() : name_("EXOTicaServer"), nh_(nullptr), executor_(nullptr)
{
}

Server::~Server()
{
    if (executor_)
    {
        executor_->cancel();
        if (spinner_thread_.joinable()) spinner_thread_.join();
    }
}

void Server::Destroy()
{
    exotica::Server::singleton_server_.reset();
}

moveit::core::RobotModelPtr LoadModelImpl(const std::string& urdf, const std::string& srdf)
{
    rdf_loader::RDFLoader loader(urdf, srdf);
    const std::shared_ptr<srdf::Model>& srdf_ = loader.getSRDF() ? loader.getSRDF() : std::shared_ptr<srdf::Model>(new srdf::Model());
    if (loader.getURDF())
    {
        return moveit::core::RobotModelPtr(new moveit::core::RobotModel(loader.getURDF(), srdf_));
    }
    else
    {
        ThrowPretty("Can't load robot model from URDF!");
    }
}

moveit::core::RobotModelPtr Server::LoadModel(const std::string& name, const std::string& urdf, const std::string& srdf)
{
    moveit::core::RobotModelPtr model;
    if (HasParam("RobotDescription"))
    {
        std::string robot_description_param;
        GetParam("RobotDescription", robot_description_param);
        RCLCPP_INFO_STREAM(GetNode()->get_logger(), "Using robot_description at " << robot_description_param);
        model = robot_model_loader::RobotModelLoader(GetNode(), robot_description_param).getModel();
    }
    else if (HasParam(GetName() + "/RobotDescription"))
    {
        std::string robot_description_param;
        GetParam(GetName() + "/RobotDescription", robot_description_param);
        RCLCPP_INFO_STREAM(GetNode()->get_logger(), "Using robot_description at " << robot_description_param);
        model = robot_model_loader::RobotModelLoader(GetNode(), robot_description_param).getModel();
    }
    else if ((urdf == "" && srdf == "") && IsRos())
    {
        model = robot_model_loader::RobotModelLoader(GetNode(), name).getModel();
    }
    // URDF and SRDF are meant to be read from files
    else if (PathExists(urdf) && PathExists(srdf))
    {
        model = LoadModelImpl(LoadFile(urdf), LoadFile(srdf));
    }
    // URDF loaded from file, SRDF empty
    else if (PathExists(urdf) && srdf == "")
    {
        model = LoadModelImpl(LoadFile(urdf), srdf);
    }
    // URDF and SRDF are passed in as strings
    else if (urdf != "" && srdf != "")
    {
        model = LoadModelImpl(urdf, srdf);
    }

    if (model)
    {
        robot_models_[name] = model;
    }
    else
    {
        ThrowPretty("Couldn't load the model at path " << name << "!");
    }
    return model;
}

void Server::GetModel(const std::string& path, moveit::core::RobotModelPtr& model, const std::string& urdf, const std::string& srdf)
{
    if (robot_models_.find(path) != robot_models_.end())
    {
        model = robot_models_[path];
    }
    else
    {
        model = LoadModel(path, urdf, srdf);
    }
}

moveit::core::RobotModelConstPtr Server::GetModel(const std::string& path, const std::string& urdf, const std::string& srdf)
{
    if (robot_models_.count(path))
    {
        return robot_models_[path];
    }
    else
    {
        return LoadModel(path, urdf, srdf);
    }
}

bool Server::HasModel(const std::string& path)
{
    return robot_models_.find(path) != robot_models_.end();
}

std::string Server::GetName()
{
    return name_;
}
}  // namespace exotica
