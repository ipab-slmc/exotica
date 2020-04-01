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

#ifdef MSGPACK_FOUND
#include <chrono>
#include <regex>

#include <exotica_core/visualization_meshcat.h>
#include <exotica_core/visualization_meshcat_types.h>

// Timeout in ms
#define TIMEOUT 10000

namespace exotica
{
inline std::vector<double> FrameToVector(const KDL::Frame& frame, double scale_x = 1.0, double scale_y = 1.0, double scale_z = 1.0)
{
    std::vector<double> ret(16);
    ret[0] = frame.M.data[0] * scale_x;
    ret[1] = frame.M.data[3];
    ret[2] = frame.M.data[6];
    ret[3] = 0.0;
    ret[4] = frame.M.data[1];
    ret[5] = frame.M.data[4] * scale_y;
    ret[6] = frame.M.data[7];
    ret[7] = 0.0;
    ret[8] = frame.M.data[2];
    ret[9] = frame.M.data[5];
    ret[10] = frame.M.data[8] * scale_z;
    ret[11] = 0.0;
    ret[12] = frame.p.data[0];
    ret[13] = frame.p.data[1];
    ret[14] = frame.p.data[2];
    ret[15] = 1.0;
    return ret;
};

inline std::vector<double> PositionToVector(const KDL::Frame& frame)
{
    std::vector<double> ret(3);
    ret[0] = frame.p.data[0];
    ret[1] = frame.p.data[1];
    ret[2] = frame.p.data[2];
    return ret;
};

inline std::vector<double> QuaternionToVector(const KDL::Frame& frame)
{
    std::vector<double> ret(4);
    frame.M.GetQuaternion(ret[0], ret[1], ret[2], ret[3]);
    return ret;
};

VisualizationMeshcat::VisualizationMeshcat(ScenePtr scene, const std::string& url, bool use_mesh_materials, const std::string& file_url) : scene_(scene), context_(1), zmq_url_(url), file_url_(file_url)
{
    HIGHLIGHT_NAMED("VisualizationMeshcat", "Initialising visualizer");
    Initialize(use_mesh_materials);
}
VisualizationMeshcat::~VisualizationMeshcat() = default;

void VisualizationMeshcat::Initialize(bool use_mesh_materials)
{
    // Connecting twice as per comment at:
    // https://github.com/rdeits/meshcat-python/blob/aa3865143120f5ace8e62aab71d825e33674d277/src/meshcat/visualizer.py#L60
    ConnectZMQ();
    web_url_ = RequestWebURL();
    if (file_url_ == "")
    {
        std::regex url_regex("(.*):(?:\\d+)(?:\\/static\\/)");
        std::smatch match;
        if (std::regex_search(web_url_, match, url_regex) && match.size() > 1)
        {
            file_url_ = match.str(1) + ":9000/files/";
        }
    }

    if (web_url_.size() > 7) file_url_ = web_url_.substr(0, web_url_.size() - 7) + "files/";
    ConnectZMQ();
    path_prefix_ = "/exotica/" + scene_->GetName() + "/";
}

void VisualizationMeshcat::ConnectZMQ()
{
    socket_ = std::unique_ptr<zmq::socket_t>(new zmq::socket_t(context_, ZMQ_REQ));
    socket_->setsockopt(ZMQ_RCVTIMEO, TIMEOUT);
    socket_->connect(zmq_url_);
}

void VisualizationMeshcat::SendZMQ(const std::string& data)
{
    zmq::message_t request(data.size());
    std::memcpy(request.data(), data.c_str(), data.size());
    socket_->send(request);
}

std::string VisualizationMeshcat::ReceiveZMQ()
{
    char buffer[2048];
    std::memset(buffer, 0, 2048);
    socket_->recv(&buffer, 2048);
    return std::string(buffer);
}

std::string VisualizationMeshcat::RequestWebURL()
{
    SendZMQ("url");
    return ReceiveZMQ();
}

std::string VisualizationMeshcat::GetWebURL()
{
    return web_url_;
}

std::string VisualizationMeshcat::GetFileURL()
{
    return file_url_;
}

template <typename T>
void VisualizationMeshcat::SendMsg(T msg)
{
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, msg);
    socket_->send(msg.type.data(), msg.type.size(), ZMQ_SNDMORE);
    socket_->send(msg.path.data(), msg.path.size(), ZMQ_SNDMORE);
    socket_->send(sbuf.data(), sbuf.size());
    ReceiveZMQ();
}

void VisualizationMeshcat::DisplayScene(bool use_mesh_materials)
{
    const std::vector<std::weak_ptr<KinematicElement>>& elements = scene_->GetKinematicTree().GetTree();

    for (std::weak_ptr<KinematicElement> weak_element : elements)
    {
        std::shared_ptr<KinematicElement> element = weak_element.lock();
        if (element->visual.size() == 0) continue;
        for (auto visual : element->visual)
        {
            if (!visual.shape) continue;
            switch (visual.shape->type)
            {
                case shapes::SPHERE:
                {
                    std::shared_ptr<shapes::Sphere> sphere = std::static_pointer_cast<shapes::Sphere>(ToStdPtr(visual.shape));

                    auto object = visualization::SetObject(path_prefix_ + visual.name,
                                                           visualization::CreateGeometryObject(visualization::GeometrySphere(sphere->radius),
                                                                                               visualization::Material(visualization::RGB(visual.color(0), visual.color(1), visual.color(2)), visual.color(3))));
                    object.object.object.matrix = FrameToVector(visual.frame);
                    SendMsg(object);
                    SendMsg(visualization::SetTransform(object.path, FrameToVector(element->frame)));
                }
                break;
                case shapes::BOX:
                {
                    std::shared_ptr<shapes::Box> box = std::static_pointer_cast<shapes::Box>(ToStdPtr(visual.shape));

                    auto object = visualization::SetObject(path_prefix_ + visual.name,
                                                           visualization::CreateGeometryObject(visualization::GeometryBox(box->size[0], box->size[1], box->size[2]),
                                                                                               visualization::Material(visualization::RGB(visual.color(0), visual.color(1), visual.color(2)), visual.color(3))));
                    object.object.object.matrix = FrameToVector(visual.frame);
                    SendMsg(object);
                    SendMsg(visualization::SetTransform(object.path, FrameToVector(element->frame)));
                }
                break;
                case shapes::CYLINDER:
                {
                    std::shared_ptr<shapes::Cylinder> cylinder = std::static_pointer_cast<shapes::Cylinder>(ToStdPtr(visual.shape));

                    auto object = visualization::SetObject(path_prefix_ + visual.name,
                                                           visualization::CreateGeometryObject(visualization::GeometryCylinder(cylinder->radius, cylinder->length),
                                                                                               visualization::Material(visualization::RGB(visual.color(0), visual.color(1), visual.color(2)), visual.color(3))));
                    // Rotate the cylinder to match meshcat convention
                    object.object.object.matrix = FrameToVector(visual.frame * KDL::Frame(KDL::Rotation::RotX(M_PI_2)));
                    SendMsg(object);
                    SendMsg(visualization::SetTransform(object.path, FrameToVector(element->frame)));
                }
                break;
                default:
                {
                    if (!visual.shape_resource_path.empty())
                    {
                        auto mesh = visualization::GeometryMesh(visual.shape_resource_path, file_url_ + visual.shape_resource_path);
                        // If using STL files or if requested specifically, use URDF colours
                        if (!use_mesh_materials || mesh.format == "stl")
                        {
                            auto object = visualization::SetObject(path_prefix_ + visual.name, visualization::CreateGeometryObject(mesh,
                                                                                                                                   visualization::Material(visualization::RGB(visual.color(0), visual.color(1), visual.color(2)), visual.color(3))));
                            object.object.object.matrix =
                                FrameToVector(visual.frame, visual.scale(0), visual.scale(1), visual.scale(2));
                            SendMsg(object);
                            SendMsg(visualization::SetTransform(object.path, FrameToVector(element->frame)));
                        }
                        else
                        {
                            auto object = visualization::SetObject(path_prefix_ + visual.name,
                                                                   visualization::CreateMeshObject(mesh,
                                                                                                   visualization::Material(visualization::RGB(visual.color(0), visual.color(1), visual.color(2)), visual.color(3))));
                            object.object.object.matrix =
                                FrameToVector(visual.frame, visual.scale(0), visual.scale(1), visual.scale(2));
                            SendMsg(object);
                            SendMsg(visualization::SetTransform(object.path, FrameToVector(element->frame)));
                        }
                    }
                    else
                    {
                        if (visual.shape->type == shapes::MESH)
                        {
                            std::shared_ptr<shapes::Mesh> shape = std::static_pointer_cast<shapes::Mesh>(visual.shape);
                            auto mesh = visualization::GeometryMeshBuffer(visual.shape);
                            auto object = visualization::SetObject(path_prefix_ + visual.name, visualization::CreateGeometryObject(mesh,
                                                                                                                                   visualization::Material(visualization::RGB(visual.color(0), visual.color(1), visual.color(2)), visual.color(3))));
                            object.object.object.matrix =
                                FrameToVector(visual.frame, visual.scale(0), visual.scale(1), visual.scale(2));
                            SendMsg(object);
                            SendMsg(visualization::SetTransform(object.path, FrameToVector(element->frame)));
                        }
                        else
                        {
                            HIGHLIGHT("Unsupported shape! " << element->segment.getName());
                        }
                    }
                }
                break;
            }
        }
    }
}

void VisualizationMeshcat::DisplayState(Eigen::VectorXdRefConst state, double t)
{
    const std::vector<std::weak_ptr<KinematicElement>>& elements = scene_->GetKinematicTree().GetTree();
    scene_->Update(state, t);

    for (std::weak_ptr<KinematicElement> weak_element : elements)
    {
        std::shared_ptr<KinematicElement> element = weak_element.lock();
        if (element->visual.size() == 0) continue;
        for (auto visual : element->visual)
        {
            SendMsg(visualization::SetTransform(path_prefix_ + visual.name, FrameToVector(element->frame)));
        }
    }
}

void VisualizationMeshcat::DisplayTrajectory(Eigen::MatrixXdRefConst trajectory, double dt)
{
    auto set_animation = visualization::SetAnimation();

    std::string clip_name = "default";
    double fps = 1.0 / dt;

    const std::vector<std::weak_ptr<KinematicElement>>& elements = scene_->GetKinematicTree().GetTree();
    for (std::weak_ptr<KinematicElement> weak_element : elements)
    {
        std::shared_ptr<KinematicElement> element = weak_element.lock();
        if (element->visual.size() == 0) continue;
        for (auto visual : element->visual)
        {
            visualization::Animation anim(path_prefix_ + visual.name);
            anim.clip = visualization::Clip(fps, clip_name);
            anim.clip.tracks.resize(2);
            anim.clip.tracks[0] = visualization::Track(".position", "vector3");
            anim.clip.tracks[1] = visualization::Track(".quaternion", "quaternion");
            anim.clip.tracks[0].keys.reserve(trajectory.rows());
            anim.clip.tracks[1].keys.reserve(trajectory.rows());
            set_animation.animations.push_back(anim);
        }
    }

    for (int t = 0; t < trajectory.rows(); ++t)
    {
        double time = static_cast<double>(t) * dt;
        scene_->Update(trajectory.row(t), time);

        int i = 0;
        for (std::weak_ptr<KinematicElement> weak_element : elements)
        {
            std::shared_ptr<KinematicElement> element = weak_element.lock();
            if (element->visual.size() == 0) continue;
            for (auto visual : element->visual)
            {
                set_animation.animations[i].clip.tracks[0].keys.push_back(visualization::Key(time, PositionToVector(element->frame)));
                set_animation.animations[i].clip.tracks[1].keys.push_back(visualization::Key(time, QuaternionToVector(element->frame)));
                ++i;
            }
        }
    }

    SendMsg(set_animation);
}

void VisualizationMeshcat::Delete(const std::string& path)
{
    SendMsg(visualization::Delete("/exotica/" + path));
}

void VisualizationMeshcat::SetProperty(const std::string& path, const std::string& property, const double& value)
{
    SendMsg(visualization::Property<double>(path, property, value));
}

void VisualizationMeshcat::SetProperty(const std::string& path, const std::string& property, const std::string& value)
{
    SendMsg(visualization::Property<std::string>(path, property, value));
}

void VisualizationMeshcat::SetProperty(const std::string& path, const std::string& property, const bool& value)
{
    SendMsg(visualization::Property<bool>(path, property, value));
}

void VisualizationMeshcat::SetProperty(const std::string& path, const std::string& property, const Eigen::Vector3d& value)
{
    std::vector<double> val(3);
    val[0] = value(0);
    val[1] = value(1);
    val[2] = value(2);
    SendMsg(visualization::Property<std::vector<double>>(path, property, val));
}

void VisualizationMeshcat::SetProperty(const std::string& path, const std::string& property, const Eigen::Vector4d& value)
{
    std::vector<double> val(4);
    val[0] = value(0);
    val[1] = value(1);
    val[2] = value(2);
    val[3] = value(3);
    SendMsg(visualization::Property<std::vector<double>>(path, property, val));
}
}  // namespace exotica
#endif  // MSGPACK_FOUND
