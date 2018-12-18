/*
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University of Edinburgh
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

#include <exotica/Scene.h>
#include <exotica/Setup.h>

#include <exotica/AttachLinkInitializer.h>
#include <exotica/LinkInitializer.h>
#include <exotica/TrajectoryInitializer.h>

namespace exotica
{
///////////////////////////////////////////////////////////////
/////////////////////// EXOTica Scene ///////////////////////
///////////////////////////////////////////////////////////////

Scene::Scene() = default;

Scene::Scene(const std::string& name) : name_(name), requestNeedsUpdating(false)
{
    object_name_ = name_;
}

Scene::~Scene() = default;

std::string Scene::getName()
{
    return name_;
}

void Scene::Instantiate(SceneInitializer& init)
{
    Object::InstatiateObject(init);
    name_ = object_name_;
    kinematica_.Debug = debug_;
    force_collision_ = init.AlwaysUpdateCollisionScene;
    robot_model::RobotModelPtr model;
    if (init.URDF == "" || init.SRDF == "")
    {
        Server::Instance()->getModel(init.RobotDescription, model);
    }
    else
    {
        Server::Instance()->getModel(init.URDF, model, init.URDF, init.SRDF);
    }
    kinematica_.Instantiate(init.JointGroup, model, name_);
    group = model->getJointModelGroup(init.JointGroup);
    ps_.reset(new planning_scene::PlanningScene(model));

    // Write URDF/SRDF to ROS param server
    if (Server::isRos() && init.SetRobotDescriptionRosParams && init.URDF != "" && init.SRDF != "")
    {
        if (debug_) HIGHLIGHT_NAMED(name_, "Setting robot_description and robot_description_semantic from URDF and SRDF initializers");
        std::string urdf_string = pathExists(init.URDF) ? loadFile(init.URDF) : init.URDF;
        std::string srdf_string = pathExists(init.SRDF) ? loadFile(init.SRDF) : init.SRDF;
        Server::setParam("/robot_description", urdf_string);
        Server::setParam("/robot_description_semantic", srdf_string);
    }

    BaseType = kinematica_.getControlledBaseType();

    if (Server::isRos())
    {
        ps_pub_ = Server::advertise<moveit_msgs::PlanningScene>(name_ + (name_ == "" ? "" : "/") + "PlanningScene", 100, true);
        proxy_pub_ = Server::advertise<visualization_msgs::Marker>(name_ + (name_ == "" ? "" : "/") + "CollisionProxies", 100, true);
        if (debug_)
            HIGHLIGHT_NAMED(
                name_,
                "Running in debug mode, planning scene will be published to '"
                    << Server::Instance()->getName() << "/" << name_
                    << "/PlanningScene'");
    }

    // Note: Using the LoadScene initializer does not support custom offsets/poses, assumes Identity transform to world_frame
    if (init.LoadScene != "")
    {
        std::vector<std::string> files = parseList(init.LoadScene, ';');
        for (const std::string& file : files) loadSceneFile(file, Eigen::Isometry3d::Identity(), false);
    }

    for (const exotica::Initializer& linkInit : init.Links)
    {
        LinkInitializer link(linkInit);
        addObject(link.Name, getFrame(link.Transform), link.Parent, nullptr, KDL::RigidBodyInertia(link.Mass, getFrame(link.CoM).p), false);
    }

    // Check list of links to exclude from CollisionScene
    if (init.RobotLinksToExcludeFromCollisionScene.size() > 0)
    {
        for (const auto& link : init.RobotLinksToExcludeFromCollisionScene)
        {
            robotLinksToExcludeFromCollisionScene_.insert(link);
            if (debug_) HIGHLIGHT_NAMED("RobotLinksToExcludeFromCollisionScene", link);
        }
    }

    collision_scene_ = Setup::createCollisionScene(init.CollisionScene);
    collision_scene_->debug_ = this->debug_;
    collision_scene_->setup();
    collision_scene_->setAlwaysExternallyUpdatedCollisionScene(force_collision_);
    collision_scene_->setReplacePrimitiveShapesWithMeshes(init.ReplacePrimitiveShapesWithMeshes);
    collision_scene_->setWorldLinkPadding(init.WorldLinkPadding);
    collision_scene_->setRobotLinkPadding(init.RobotLinkPadding);
    collision_scene_->setWorldLinkScale(init.WorldLinkScale);
    collision_scene_->setRobotLinkScale(init.RobotLinkScale);
    collision_scene_->replaceCylindersWithCapsules = init.ReplaceCylindersWithCapsules;
    updateSceneFrames();
    updateInternalFrames(false);

    for (const exotica::Initializer& linkInit : init.AttachLinks)
    {
        AttachLinkInitializer link(linkInit);
        if (link.Local)
        {
            attachObjectLocal(link.Name, link.Parent, getFrame(link.Transform));
        }
        else
        {
            attachObject(link.Name, link.Parent);
        }
    }

    AllowedCollisionMatrix acm;
    std::vector<std::string> acm_names;
    ps_->getAllowedCollisionMatrix().getAllEntryNames(acm_names);
    for (auto& name1 : acm_names)
    {
        for (auto& name2 : acm_names)
        {
            collision_detection::AllowedCollision::Type type = collision_detection::AllowedCollision::Type::ALWAYS;
            ps_->getAllowedCollisionMatrix().getAllowedCollision(name1, name2, type);
            if (type == collision_detection::AllowedCollision::Type::ALWAYS)
            {
                acm.setEntry(name1, name2);
            }
        }
    }
    collision_scene_->setACM(acm);

    for (const exotica::Initializer& it : init.Trajectories)
    {
        TrajectoryInitializer trajInit(it);
        if (trajInit.File != "")
        {
            addTrajectoryFromFile(trajInit.Link, trajInit.File);
        }
        else
        {
            addTrajectory(trajInit.Link, trajInit.Trajectory);
        }
    }

    if (debug_) INFO_NAMED(name_, "Exotica Scene initialized");
}

void Scene::requestKinematics(KinematicsRequest& request, std::function<void(std::shared_ptr<KinematicResponse>)> callback)
{
    kinematicRequest = request;
    kinematicRequestCallback = callback;
    kinematicSolution = kinematica_.RequestFrames(kinematicRequest);
    kinematicRequestCallback(kinematicSolution);
    requestNeedsUpdating = false;
}

void Scene::updateTrajectoryGenerators(double t)
{
    for (auto& it : trajectory_generators_)
    {
        it.second.first.lock()->GeneratedOffset = it.second.second->getPosition(t);
    }
}

void Scene::Update(Eigen::VectorXdRefConst x, double t)
{
    if (requestNeedsUpdating && kinematicRequestCallback)
    {
        updateInternalFrames();
    }

    updateTrajectoryGenerators(t);
    kinematica_.Update(x);
    if (force_collision_) collision_scene_->updateCollisionObjectTransforms();
    if (debug_) publishScene();
}

void Scene::updateMoveItPlanningScene()
{
    std::map<std::string, double> modelState = getModelStateMap();
    for (const auto& joint : modelState)
    {
        try
        {
            ps_->getCurrentStateNonConst().setVariablePosition(joint.first, joint.second);
        }
        catch (const std::out_of_range& e)
        {
            HIGHLIGHT("Could not find Kinematica joint name in MoveIt: " + joint.first);
        }
    }

    // The floating base joint in MoveIt uses quaternion, while Kinematica uses
    // RPY [but using rot_x, rot_y, and rot_z as joint names]. Thus, we need to
    // fix the orientation of the virtual floating base by extracting the RPY
    // values, converting them to quaternion, and then updating the planning
    // scene.
    if (kinematica_.getModelBaseType() == BASE_TYPE::FLOATING)
    {
        KDL::Rotation rot = KDL::Rotation::RPY(modelState[kinematica_.getRootJointName() + "/rot_x"], modelState[kinematica_.getRootJointName() + "/rot_y"], modelState[kinematica_.getRootJointName() + "/rot_z"]);
        Eigen::Quaterniond quat(Eigen::Map<const Eigen::Matrix3d>(rot.data).transpose());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.getRootJointName() + "/rot_x", quat.x());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.getRootJointName() + "/rot_y", quat.y());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.getRootJointName() + "/rot_z", quat.z());
        ps_->getCurrentStateNonConst().setVariablePosition(kinematica_.getRootJointName() + "/rot_w", quat.w());
    }
}

void Scene::publishScene()
{
    if (Server::isRos())
    {
        ps_pub_.publish(getPlanningSceneMsg());
    }
}

void Scene::publishProxies(const std::vector<CollisionProxy>& proxies)
{
    if (Server::isRos())
    {
        proxy_pub_.publish(proxyToMarker(proxies, kinematica_.getRootFrameName()));
    }
}

visualization_msgs::Marker Scene::proxyToMarker(const std::vector<CollisionProxy>& proxies, const std::string& frame)
{
    visualization_msgs::Marker ret;
    ret.header.frame_id = "exotica/" + frame;
    ret.action = visualization_msgs::Marker::ADD;
    ret.frame_locked = false;
    ret.ns = "Proxies";
    ret.color.a = 1.0;
    ret.id = 0;
    ret.type = visualization_msgs::Marker::LINE_LIST;
    ret.points.resize(proxies.size() * 6);
    ret.colors.resize(proxies.size() * 6);
    ret.scale.x = 0.005;
    double normalLength = 0.01;
    std_msgs::ColorRGBA normal = getColor(0.8, 0.8, 0.8);
    std_msgs::ColorRGBA far = getColor(0.5, 0.5, 0.5);
    std_msgs::ColorRGBA colliding = getColor(1, 0, 0);
    for (int i = 0; i < proxies.size(); i++)
    {
        KDL::Vector c1 = KDL::Vector(proxies[i].contact1(0), proxies[i].contact1(1), proxies[i].contact1(2));
        KDL::Vector c2 = KDL::Vector(proxies[i].contact2(0), proxies[i].contact2(1), proxies[i].contact2(2));
        KDL::Vector n1 = KDL::Vector(proxies[i].normal1(0), proxies[i].normal1(1), proxies[i].normal1(2));
        KDL::Vector n2 = KDL::Vector(proxies[i].normal2(0), proxies[i].normal2(1), proxies[i].normal2(2));
        tf::pointKDLToMsg(c1, ret.points[i * 6]);
        tf::pointKDLToMsg(c2, ret.points[i * 6 + 1]);
        tf::pointKDLToMsg(c1, ret.points[i * 6 + 2]);
        tf::pointKDLToMsg(c1 + n1 * normalLength, ret.points[i * 6 + 3]);
        tf::pointKDLToMsg(c2, ret.points[i * 6 + 4]);
        tf::pointKDLToMsg(c2 + n2 * normalLength, ret.points[i * 6 + 5]);
        ret.colors[i * 6] = ret.colors[i * 6 + 1] = proxies[i].distance > 0 ? far : colliding;
        ret.colors[i * 6 + 2] = ret.colors[i * 6 + 3] = ret.colors[i * 6 + 4] = ret.colors[i * 6 + 5] = normal;
    }
    return ret;
}

void Scene::setCollisionScene(const moveit_msgs::PlanningScene& scene)
{
    ps_->usePlanningSceneMsg(scene);
    updateSceneFrames();
    updateInternalFrames();
}

void Scene::updateWorld(const moveit_msgs::PlanningSceneWorldConstPtr& world)
{
    ps_->processPlanningSceneWorldMsg(*world);
    updateSceneFrames();
    updateInternalFrames();
}

void Scene::updateCollisionObjects()
{
    collision_scene_->updateCollisionObjects(kinematica_.getCollisionTreeMap());
}

CollisionScene_ptr& Scene::getCollisionScene()
{
    return collision_scene_;
}

std::string Scene::getRootFrameName()
{
    return kinematica_.getRootFrameName();
}

std::string Scene::getRootJointName()
{
    return kinematica_.getRootJointName();
}

moveit_msgs::PlanningScene Scene::getPlanningSceneMsg()
{
    // Update the joint positions in the PlanningScene from Kinematica - we do
    // not do this on every Update() as it is only required when publishing
    // the scene and would take unnecessary time otherwise.
    updateMoveItPlanningScene();

    moveit_msgs::PlanningScene msg;
    ps_->getPlanningSceneMsg(msg);

    // The robot link paddings and scales are applied in the CollisionScene
    // and are not propagated back to the internal MoveIt PlanningScene. Here
    // we set them in the message that is being returned.
    msg.link_padding.clear();
    msg.link_scale.clear();
    for (auto robot_link : msg.robot_state.joint_state.name)
    {
        moveit_msgs::LinkPadding padding;
        padding.link_name = robot_link;
        padding.padding = collision_scene_->getRobotLinkPadding();
        msg.link_padding.push_back(padding);

        moveit_msgs::LinkScale scale;
        scale.link_name = robot_link;
        scale.scale = collision_scene_->getRobotLinkScale();
        msg.link_scale.push_back(scale);
    }

    // As we cannot apply world link scalings in the message itself, we need to
    // manually scale the objects.
    // TODO(wxm): Recreate as updated poses won't be reflected (e.g. trajectories)
    if (collision_scene_->getWorldLinkScale() != 1.0 || collision_scene_->getWorldLinkPadding() > 0.0)
    {
        for (auto it : msg.world.collision_objects)
        {
            // Primitives
            for (auto primitive : it.primitives)
            {
                shapes::ShapePtr tmp(shapes::constructShapeFromMsg(primitive));
                tmp->scaleAndPadd(collision_scene_->getWorldLinkScale(), collision_scene_->getWorldLinkPadding());
                shapes::ShapeMsg tmp_msg;
                shapes::constructMsgFromShape(const_cast<shapes::Shape*>(tmp.get()), tmp_msg);
                primitive = boost::get<shape_msgs::SolidPrimitive>(tmp_msg);
            }

            // Meshes
            for (auto mesh : it.meshes)
            {
                shapes::ShapePtr tmp(shapes::constructShapeFromMsg(mesh));
                tmp->scaleAndPadd(collision_scene_->getWorldLinkScale(), collision_scene_->getWorldLinkPadding());
                shapes::ShapeMsg tmp_msg;
                shapes::constructMsgFromShape(const_cast<shapes::Shape*>(tmp.get()), tmp_msg);
                mesh = boost::get<shape_msgs::Mesh>(tmp_msg);
            }

            // NB: Scaling and padding does not apply to planes
        }
    }

    return msg;
}

exotica::KinematicTree& Scene::getKinematicTree()
{
    return kinematica_;
}

void Scene::getJointNames(std::vector<std::string>& joints)
{
    joints = kinematica_.getJointNames();
}

std::vector<std::string> Scene::getJointNames()
{
    return kinematica_.getJointNames();
}

std::vector<std::string> Scene::getControlledLinkNames()
{
    return kinematica_.getControlledLinkNames();
}

std::vector<std::string> Scene::getModelLinkNames()
{
    return kinematica_.getModelLinkNames();
}

std::vector<std::string> Scene::getModelJointNames()
{
    return kinematica_.getModelJointNames();
}

Eigen::VectorXd Scene::getModelState()
{
    return kinematica_.getModelState();
}

std::map<std::string, double> Scene::getModelStateMap()
{
    return kinematica_.getModelStateMap();
}

void Scene::setModelState(Eigen::VectorXdRefConst x, double t, bool updateTraj)
{
    if (requestNeedsUpdating && kinematicRequestCallback)
    {
        updateInternalFrames();
    }

    if (updateTraj) updateTrajectoryGenerators(t);
    // Update Kinematica internal state
    kinematica_.setModelState(x);

    if (force_collision_) collision_scene_->updateCollisionObjectTransforms();
    if (debug_) publishScene();
}

void Scene::setModelState(std::map<std::string, double> x, double t, bool updateTraj)
{
    if (requestNeedsUpdating && kinematicRequestCallback)
    {
        updateInternalFrames();
    }

    if (updateTraj) updateTrajectoryGenerators(t);
    // Update Kinematica internal state
    kinematica_.setModelState(x);

    if (force_collision_) collision_scene_->updateCollisionObjectTransforms();
    if (debug_) publishScene();
}

Eigen::VectorXd Scene::getControlledState()
{
    return kinematica_.getControlledState();
}

std::string Scene::getGroupName()
{
    return group->getName();
}

void Scene::loadScene(const std::string& scene, const KDL::Frame& offset, bool updateCollisionScene)
{
    Eigen::Isometry3d tmp_offset;
    tf::transformKDLToEigen(offset, tmp_offset);
    loadScene(scene, tmp_offset, updateCollisionScene);
}

void Scene::loadScene(const std::string& scene, const Eigen::Isometry3d& offset, bool updateCollisionScene)
{
    std::stringstream ss(scene);
    loadSceneFromStringStream(ss, offset, updateCollisionScene);
}

void Scene::loadSceneFile(const std::string& file_name, const KDL::Frame& offset, bool updateCollisionScene)
{
    Eigen::Isometry3d tmp_offset;
    tf::transformKDLToEigen(offset, tmp_offset);
    loadSceneFile(file_name, tmp_offset, updateCollisionScene);
}

void Scene::loadSceneFile(const std::string& file_name, const Eigen::Isometry3d& offset, bool updateCollisionScene)
{
    std::ifstream ss(parsePath(file_name));
    if (!ss.is_open()) throw_pretty("Cant read file '" << parsePath(file_name) << "'!");
    loadSceneFromStringStream(ss, offset, updateCollisionScene);
}

void Scene::loadSceneFromStringStream(std::istream& in, const Eigen::Isometry3d& offset, bool updateCollisionScene)
{
#if ROS_VERSION_MINIMUM(1, 14, 0)  // if ROS version >= ROS_MELODIC
    ps_->loadGeometryFromStream(in, offset);
#else
    ps_->loadGeometryFromStream(in, Eigen::Affine3d(offset));
#endif

    updateSceneFrames();
    if (updateCollisionScene) updateInternalFrames();
}

std::string Scene::getScene()
{
    std::stringstream ss;
    ps_->saveGeometryToStream(ss);
    // TODO: include all custom environment scene objects
    return ss.str();
}

void Scene::cleanScene()
{
    ps_->removeAllCollisionObjects();
    // TODO: remove all custom environment scene objects
    updateSceneFrames();
}

void Scene::updateInternalFrames(bool updateRequest)
{
    for (auto& it : custom_links_)
    {
        Eigen::Isometry3d pose;
        tf::transformKDLToEigen(it->Segment.getFrameToTip(), pose);
        std::string shapeResourcePath = it->ShapeResourcePath;
        Eigen::Vector3d scale = it->Scale;
        it = kinematica_.AddElement(it->Segment.getName(), pose, it->ParentName, it->Shape, it->Segment.getInertia(), Eigen::Vector4d::Zero(), it->IsControlled);
        it->ShapeResourcePath = shapeResourcePath;
        it->Scale = scale;
    }

    auto trajCopy = trajectory_generators_;
    trajectory_generators_.clear();
    for (auto& traj : trajCopy)
    {
        addTrajectory(traj.first, traj.second.second);
    }

    for (auto& link : attached_objects_)
    {
        attachObjectLocal(link.first, link.second.Parent, link.second.Pose);
    }

    kinematica_.UpdateModel();

    if (updateRequest)
    {
        kinematicSolution = kinematica_.RequestFrames(kinematicRequest);
        kinematicRequestCallback(kinematicSolution);
    }

    updateCollisionObjects();

    requestNeedsUpdating = false;
}

void Scene::updateSceneFrames()
{
    kinematica_.resetModel();

    // Add world objects
    for (const auto& object : *ps_->getWorld())
    {
        if (object.second->shapes_.size())
        {
            // Use the first collision shape as the origin of the object
            Eigen::Isometry3d objTransform;
            objTransform.translation() = object.second->shape_poses_[0].translation();
            objTransform.linear() = object.second->shape_poses_[0].rotation();
            kinematica_.AddEnvironmentElement(object.first, objTransform);

            for (int i = 0; i < object.second->shape_poses_.size(); i++)
            {
                Eigen::Isometry3d shapeTransform;
                shapeTransform.translation() = object.second->shape_poses_[i].translation();
                shapeTransform.linear() = object.second->shape_poses_[i].rotation();
                Eigen::Isometry3d trans = objTransform.inverse() * shapeTransform;
                if (ps_->hasObjectColor(object.first))
                {
                    auto colorMsg = ps_->getObjectColor(object.first);
                    Eigen::Vector4d color = Eigen::Vector4d(colorMsg.r, colorMsg.g, colorMsg.b, colorMsg.a);
                    kinematica_.AddEnvironmentElement(object.first + "_collision_" + std::to_string(i), trans, object.first, object.second->shapes_[i], KDL::RigidBodyInertia::Zero(), color);
                }
                else
                {
                    kinematica_.AddEnvironmentElement(object.first + "_collision_" + std::to_string(i), trans, object.first, object.second->shapes_[i]);
                }
            }
        }
        else
        {
            HIGHLIGHT("Object with no shapes ('" << object.first << "')");
        }
    }

    // Add robot collision objects
    ps_->getCurrentStateNonConst().update(true);
    const std::vector<const robot_model::LinkModel*>& links =
        ps_->getCollisionRobot()->getRobotModel()->getLinkModelsWithCollisionGeometry();
    int lastControlledJointId = -1;
    std::string lastControlledLinkName;
    modelLink_to_collisionLink_map_.clear();
    modelLink_to_collisionElement_map_.clear();
    controlledLink_to_collisionLink_map_.clear();
    for (int i = 0; i < links.size(); ++i)
    {
        // Check whether link is excluded from collision checking
        if (robotLinksToExcludeFromCollisionScene_.find(links[i]->getName()) != robotLinksToExcludeFromCollisionScene_.end())
        {
            continue;
        }

        Eigen::Isometry3d objTransform;
        objTransform.translation() = ps_->getCurrentState().getGlobalLinkTransform(links[i]).translation();
        objTransform.linear() = ps_->getCurrentState().getGlobalLinkTransform(links[i]).rotation();

        int jointId = getKinematicTree().IsControlledLink(links[i]->getName());
        if (jointId != -1)
        {
            if (lastControlledJointId != jointId)
            {
                lastControlledLinkName = links[i]->getName();
                lastControlledJointId = jointId;
            }
        }

        for (int j = 0; j < links[i]->getShapes().size(); ++j)
        {
            // // Workaround for issue #172
            // // To disable collisions with visual tags, we define a sphere of radius 0 which we will ignore here
            // // This is in contrast to IHMC's convention where a sphere of radius 0 in the SDF is a desired contact point
            // if (links[i]->getShapes()[j]->type == shapes::ShapeType::SPHERE)
            //     if (static_cast<const shapes::Sphere*>(links[i]->getShapes()[j].get())->radius == 0.0)
            //         continue;

            Eigen::Affine3d collisionBodyTransform_affine = ps_->getCurrentState().getCollisionBodyTransform(links[i], j);
            Eigen::Isometry3d collisionBodyTransform;
            collisionBodyTransform.translation() = collisionBodyTransform_affine.translation();
            collisionBodyTransform.linear() = collisionBodyTransform_affine.rotation();
            Eigen::Isometry3d trans = objTransform.inverse(Eigen::Isometry) * collisionBodyTransform;

            std::shared_ptr<KinematicElement> element = kinematica_.AddElement(links[i]->getName() + "_collision_" + std::to_string(j), trans, links[i]->getName(), links[i]->getShapes()[j]);
            modelLink_to_collisionElement_map_[links[i]->getName()].push_back(element);

            // Set up mappings
            modelLink_to_collisionLink_map_[links[i]->getName()].push_back(links[i]->getName() + "_collision_" + std::to_string(j));

            if (lastControlledLinkName != "")
            {
                controlledLink_to_collisionLink_map_[lastControlledLinkName].push_back(links[i]->getName() + "_collision_" + std::to_string(j));
            }
        }
    }

    kinematica_.UpdateModel();

    requestNeedsUpdating = true;
}

void Scene::addObject(const std::string& name, const KDL::Frame& transform, const std::string& parent, shapes::ShapeConstPtr shape, const KDL::RigidBodyInertia& inertia, bool updateCollisionScene)
{
    if (kinematica_.doesLinkWithNameExist(name)) throw_pretty("Link '" << name << "' already exists in the scene!");
    std::string parent_name = (parent == "") ? kinematica_.getRootFrameName() : parent;
    if (!kinematica_.doesLinkWithNameExist(parent_name)) throw_pretty("Can't find parent '" << parent_name << "'!");
    Eigen::Isometry3d pose;
    tf::transformKDLToEigen(transform, pose);
    custom_links_.push_back(kinematica_.AddElement(name, pose, parent_name, shape, inertia));
    if (updateCollisionScene) updateCollisionObjects();
}

void Scene::addObject(const std::string& name, const KDL::Frame& transform, const std::string& parent, const std::string& shapeResourcePath, Eigen::Vector3d scale, const KDL::RigidBodyInertia& inertia, bool updateCollisionScene)
{
    if (kinematica_.doesLinkWithNameExist(name)) throw_pretty("Link '" << name << "' already exists in the scene!");
    std::string parent_name = (parent == "") ? kinematica_.getRootFrameName() : parent;
    if (!kinematica_.doesLinkWithNameExist(parent_name)) throw_pretty("Can't find parent '" << parent_name << "'!");
    Eigen::Isometry3d pose;
    tf::transformKDLToEigen(transform, pose);
    custom_links_.push_back(kinematica_.AddElement(name, pose, parent_name, shapeResourcePath, scale, inertia));
    updateSceneFrames();
    updateInternalFrames();
    if (updateCollisionScene) updateCollisionObjects();
}

void Scene::addObjectToEnvironment(const std::string& name, const KDL::Frame& transform, shapes::ShapeConstPtr shape, const std_msgs::ColorRGBA& colour, const bool updateCollisionScene)
{
    if (kinematica_.hasModelLink(name))
    {
        throw std::runtime_error("link '" + name + "' already exists in kinematic tree");
    }
    Eigen::Isometry3d pose;
    tf::transformKDLToEigen(transform, pose);
    ps_->getWorldNonConst()->addToObject(name, shape, pose);
    ps_->setObjectColor(name, colour);
    updateSceneFrames();
    if (updateCollisionScene) updateInternalFrames();
}

void Scene::removeObject(const std::string& name)
{
    auto it = std::begin(custom_links_);
    while (it != std::end(custom_links_))
    {
        if ((*it)->Segment.getName() == name)
        {
            custom_links_.erase(it);
            updateSceneFrames();
            updateInternalFrames();
            return;
        }
        else
        {
            ++it;
        }
    }
    throw_pretty("Link " << name << " not removed as it cannot be found.");
}

void Scene::attachObject(const std::string& name, const std::string& parent)
{
    kinematica_.changeParent(name, parent, KDL::Frame::Identity(), false);
    attached_objects_[name] = AttachedObject(parent);
}

void Scene::attachObjectLocal(const std::string& name, const std::string& parent, const KDL::Frame& pose)
{
    kinematica_.changeParent(name, parent, pose, true);
    attached_objects_[name] = AttachedObject(parent, pose);
}

void Scene::detachObject(const std::string& name)
{
    if (!hasAttachedObject(name)) throw_pretty("'" << name << "' is not attached to the robot!");
    auto object = attached_objects_.find(name);
    kinematica_.changeParent(name, "", KDL::Frame::Identity(), false);
    attached_objects_.erase(object);
}

bool Scene::hasAttachedObject(const std::string& name)
{
    return attached_objects_.find(name) != attached_objects_.end();
}

void Scene::addTrajectoryFromFile(const std::string& link, const std::string& traj)
{
    addTrajectory(link, loadFile(traj));
}

void Scene::addTrajectory(const std::string& link, const std::string& traj)
{
    addTrajectory(link, std::shared_ptr<Trajectory>(new Trajectory(traj)));
}

void Scene::addTrajectory(const std::string& link, std::shared_ptr<Trajectory> traj)
{
    const auto& tree = kinematica_.getTreeMap();
    const auto& it = tree.find(link);
    if (it == tree.end()) throw_pretty("Can't find link '" << link << "'!");
    if (traj->getDuration() == 0.0) throw_pretty("The trajectory is empty!");
    trajectory_generators_[link] = std::pair<std::weak_ptr<KinematicElement>, std::shared_ptr<Trajectory>>(it->second, traj);
    it->second.lock()->IsTrajectoryGenerated = true;
}

std::shared_ptr<Trajectory> Scene::getTrajectory(const std::string& link)
{
    const auto& it = trajectory_generators_.find(link);
    if (it == trajectory_generators_.end()) throw_pretty("No trajectory generator defined for link '" << link << "'!");
    return it->second.second;
}

void Scene::removeTrajectory(const std::string& link)
{
    const auto& it = trajectory_generators_.find(link);
    if (it == trajectory_generators_.end()) throw_pretty("No trajectory generator defined for link '" << link << "'!");
    it->second.first.lock()->IsTrajectoryGenerated = false;
    trajectory_generators_.erase(it);
}
}
//  namespace exotica
