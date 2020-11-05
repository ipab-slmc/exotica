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

#include <boost/any.hpp>
#include <typeinfo>

#include <exotica_core/server.h>
#include <exotica_core/tools.h>

exotica::ServerPtr exotica::Server::singleton_server_ = nullptr;
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

void Server::Destroy()
{
    exotica::Server::singleton_server_.reset();
}

robot_model::RobotModelPtr LoadModelImpl(const std::string& urdf, const std::string& srdf)
{
    rdf_loader::RDFLoader loader(urdf, srdf);
#if ROS_VERSION_MINIMUM(1, 14, 0)  // if ROS version >= ROS_MELODIC
    const std::shared_ptr<srdf::Model>& srdf_ = loader.getSRDF() ? loader.getSRDF() : std::shared_ptr<srdf::Model>(new srdf::Model());
#else
    const boost::shared_ptr<srdf::Model>& srdf_ = loader.getSRDF() ? loader.getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
#endif
    if (loader.getURDF())
    {
        return robot_model::RobotModelPtr(new robot_model::RobotModel(loader.getURDF(), srdf_));
    }
    else
    {
        ThrowPretty("Can't load robot model from URDF!");
    }
}

robot_model::RobotModelPtr Server::LoadModel(const std::string& name, const std::string& urdf, const std::string& srdf)
{
    robot_model::RobotModelPtr model;
    if (HasParam("RobotDescription"))
    {
        std::string robot_description_param;
        GetParam("RobotDescription", robot_description_param);
        ROS_INFO_STREAM("Using robot_description at " << robot_description_param);
        model = robot_model_loader::RobotModelLoader(robot_description_param, false).getModel();
    }
    else if (HasParam(GetName() + "/RobotDescription"))
    {
        std::string robot_description_param;
        GetParam(GetName() + "/RobotDescription", robot_description_param);
        ROS_INFO_STREAM("Using robot_description at " << robot_description_param);
        model = robot_model_loader::RobotModelLoader(robot_description_param, false).getModel();
    }
    else if ((urdf == "" && srdf == "") && IsRos())
    {
        model = robot_model_loader::RobotModelLoader(name, false).getModel();
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

void Server::GetModel(const std::string& path, robot_model::RobotModelPtr& model, const std::string& urdf, const std::string& srdf)
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

robot_model::RobotModelConstPtr Server::GetModel(const std::string& path, const std::string& urdf, const std::string& srdf)
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
}
