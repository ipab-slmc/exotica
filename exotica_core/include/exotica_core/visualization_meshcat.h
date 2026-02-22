//
// Copyright (c) 2019
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

#ifndef EXOTICA_CORE_VISUALIZATION_MESHCAT_H_
#define EXOTICA_CORE_VISUALIZATION_MESHCAT_H_

#include <memory>
#include <zmq.hpp>

#include <exotica_core/scene.h>
#include <exotica_core/tools/uncopyable.h>

namespace exotica
{
class VisualizationMeshcat : public Uncopyable
{
public:
    VisualizationMeshcat(ScenePtr scene, const std::string& url, bool use_mesh_materials = true, const std::string& file_url = "");
    virtual ~VisualizationMeshcat();

    void Initialize(bool use_mesh_materials);

    void DisplayScene(bool use_mesh_materials = true);
    void DisplayState(Eigen::VectorXdRefConst state, double t = 0.0);
    void DisplayTrajectory(Eigen::MatrixXdRefConst trajectory, double dt = 1.0);
    void Delete(const std::string& path = "");
    void SetProperty(const std::string& path, const std::string& property, const double& value);
    void SetProperty(const std::string& path, const std::string& property, const std::string& value);
    void SetProperty(const std::string& path, const std::string& property, const bool& value);
    void SetProperty(const std::string& path, const std::string& property, const Eigen::Vector3d& value);
    void SetProperty(const std::string& path, const std::string& property, const Eigen::Vector4d& value);
    std::string GetWebURL();
    std::string GetFileURL();

private:
    ScenePtr scene_ = std::make_shared<Scene>(nullptr);

    void ConnectZMQ();
    void SendZMQ(const std::string& data);
    std::string ReceiveZMQ();
    std::string RequestWebURL();

    template <typename T>
    void SendMsg(T msg);

    std::string zmq_url_;
    std::string web_url_;
    std::string file_url_;

    std::string path_prefix_;

    zmq::context_t context_;
    std::unique_ptr<zmq::socket_t> socket_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_VISUALIZATION_MESHCAT_H_
