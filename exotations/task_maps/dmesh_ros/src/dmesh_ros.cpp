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

#include <dmesh_ros/dmesh_ros.h>
#include <dmesh_ros/DMeshVertexInitializer.h>
REGISTER_TASKMAP_TYPE("DMeshROS", exotica::DMeshROS);

namespace exotica
{
    DMeshROS::DMeshROS() : ir_(0.2)
    {
    }

    DMeshROS::~DMeshROS()
    {
    }

    void DMeshROS::Instantiate(DMeshROSInitializer& init)
    {
        init_ = init;
    }

    void DMeshROS::assignScene(Scene_ptr scene)
    {
        scene_ = scene;
        Initialize();
    }

    void DMeshROS::Initialize()
    {
        q_size_ = scene_->getSolver().getNumJoints();
        if(init_.Size>0)
        {
            size_ = init_.Size;
        }
        else
        {
            size_ = Frames.size();
        }
        kp_ = init_.PoseGain;
        ko_ = init_.ObstacleGain;
        kg_ = init_.GoalGain;
        usePose_ = init_.UsePose;
        wo_ = 10;
        wg_ = 10;

        radius_.resize(Frames.size());
        links_.resize(Frames.size());
        link_types_.resize(Frames.size());

        std::vector<DMeshVertexInitializer> verts(Frames.size());

        for(int i=0;i<Frames.size();i++)
        {
            verts[i] = DMeshVertexInitializer(init_.EndEffector[i]);
        }

        gManager_.initialisation(verts, size_);

        robot_size_ = Frames.size();

        if (robot_size_>size_) throw_named("The size of DMesh must be larger than number of vertices! " << robot_size_);

        ext_size_ = size_ - robot_size_;

        if (usePose_)
        {
            task_size_ = (size_) * (size_ - 1) / 2 - robot_size_;
        }
        else
        {
            task_size_ = robot_size_ * ext_size_;
        }
        obs_close_.resize(ext_size_);

        if(debug_)  HIGHLIGHT_NAMED("DMeshROS", "Distance Mesh (ROS) has been initialised: Maximum Graph size="<<size_<<", Robot link size="<<robot_size_<<", Unconnected external object size="<<ext_size_);
    }

    void DMeshROS::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
    {
        if(phi.rows() != task_size_) throw_named("Wrong size of phi!");
        phi = computeLaplace();
    }

    void DMeshROS::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
    {
        if(phi.rows() != task_size_) throw_named("Wrong size of phi!");
        if(J.rows() != task_size_ || J.cols() != Kinematics.J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics.J(0).data.cols());
        phi = computeLaplace();
        J = computeJacobian();
    }

    int DMeshROS::taskSpaceDim()
    {
        //task_size_ = gManager_.getGraph()->getCurrentSize();
        return task_size_;
    }

    Eigen::VectorXd DMeshROS::getGoalLaplace()
    {
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(task_size_);
        updateGraphFromKS();

        Eigen::MatrixXd dist;
        if (!gManager_.getGraph()->getGoalDistanceEigen(dist))
        {
            throw_named("Failed to get distances!");
        }

        int cnt = 0;
        for (int j = 0; j < robot_size_; j++)
        {
            int tmp = robot_size_;
            if (usePose_) tmp = j + 2;
            for (int l = tmp; l < size_; l++)
            {
                switch (gManager_.getGraph()->getVertex(l)->getType())
                {
                case VERTEX_TYPE::LINK:
                    goal(cnt) = kp_ * dist(j, l);
                    break;
                case VERTEX_TYPE::OBSTACLE:
                    if (gManager_.getGraph()->getVertex(l)->checkList(links_[j]))
                    {
                        goal(cnt) = ko_;
                    }
                    break;
                case VERTEX_TYPE::OBSTACLE_TO_ALL:
                    goal(cnt) = ko_;
                    break;
                case VERTEX_TYPE::GOAL:
                    goal(cnt) = gManager_.getGraph()->getVertex(l)->w_* gManager_.getGraph()->getVertex(l)->radius_;
                    break;
                default:
                    //	All other types will zero for goal laplace
                    break;
                }
                cnt++;
            }
        }
        return goal;
    }

    Eigen::VectorXd DMeshROS::computeLaplace()
    {

        updateGraphFromKS();
        if (!gManager_.getGraph()->getAcutalDistanceEigen(dist_))
        {
            throw_named("Couldn't get actual distances!");
        }

        Eigen::VectorXd Phi = Eigen::VectorXd::Zero(task_size_);
        int cnt = 0;
        double b = 0, d = 0;
        for (int j = 0; j < robot_size_; j++)
        {
            int tmp = robot_size_;
            if (usePose_) tmp = j + 2;
            for (int l = tmp; l < size_; l++)
            {
                b = d = 0;
                switch (gManager_.getGraph()->getVertex(l)->getType())
                {
                case VERTEX_TYPE::LINK:
                    Phi(cnt) = kp_ * dist_(j, l);
                    break;
                case VERTEX_TYPE::OBSTACLE:
                    if (gManager_.getGraph()->getVertex(l)->checkList(links_[j]))
                    {
                        if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
                                - gManager_.getGraph()->getVertex(l)->getRadius() < 0.05)
                            Phi(cnt) = ko_ * (1 - exp(-wo_ * dist_(j, l)));
                        else
                            Phi(cnt) = ko_;

                    }
                    break;
                case VERTEX_TYPE::OBSTACLE_TO_ALL:
                    if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
                            - gManager_.getGraph()->getVertex(l)->getRadius() < 0.05)
                        Phi(cnt) = ko_ * (1 - exp(-wo_ * dist_(j, l)));
                    else
                        Phi(cnt) = ko_;
                    break;
                case VERTEX_TYPE::GOAL:
                    Phi(cnt) = gManager_.getGraph()->getVertex(l)->w_ * dist_(j, l);
                    break;
                default:
                    break;
                }
                cnt++;
            }
        }
        HIGHLIGHT(cnt);
        return Phi;
    }

    Eigen::MatrixXd DMeshROS::computeJacobian()
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(task_size_, q_size_);
        double d_ = 0;
        int cnt;
        for (int i = 0; i < q_size_; i++)
        {
            cnt = 0;
            for (int j = 0; j < robot_size_; j++)
            {
                int tmp = robot_size_;
                if (usePose_) tmp = j + 2;
                for (int l = tmp; l < size_; l++)
                {
                    if (dist_(j, l) > 0)
                    {
                        switch (gManager_.getGraph()->getVertex(l)->getType())
                        {
                        case VERTEX_TYPE::LINK:
                            d_ = ((gManager_.getGraph()->getVertex(j)->position_
                                   - gManager_.getGraph()->getVertex(l)->position_).dot(
                                      Eigen::Vector3d(
                                          Kinematics.J[j].data.block(0, i, 3, 1)
                                          - Kinematics.J[l].data.block(0, i, 3, 1)))) / dist_(j, l);
                            J(cnt, i) = kp_ * d_;
                            break;
                        case VERTEX_TYPE::OBSTACLE:
                            if (gManager_.getGraph()->getVertex(l)->checkList(
                                        links_[j]))
                            {
                                if (dist_(j, l)
                                        - gManager_.getGraph()->getVertex(j)->getRadius()
                                        - gManager_.getGraph()->getVertex(l)->getRadius() < 0.5)
                                {
                                    d_ = ((gManager_.getGraph()->getVertex(j)->position_
                                           - gManager_.getGraph()->getVertex(l)->position_).dot(
                                              Eigen::Vector3d(Kinematics.J[j].data.block(0, i, 3, 1))))
                                            / dist_(j, l);
                                    J(cnt, i) = ko_ * wo_ * d_ * exp(-wo_ * dist_(j, l));
                                }
                            }
                            break;
                        case VERTEX_TYPE::OBSTACLE_TO_ALL:
                            if (dist_(j, l) - gManager_.getGraph()->getVertex(j)->getRadius()
                                    - gManager_.getGraph()->getVertex(l)->getRadius() < 0.5)
                            {
                                d_ = ((gManager_.getGraph()->getVertex(j)->position_
                                       - gManager_.getGraph()->getVertex(l)->position_).dot(
                                          Eigen::Vector3d(Kinematics.J[j].data.block(0, i, 3, 1))))
                                        / dist_(j, l);
                                J(cnt, i) = ko_ * wo_ * d_ * exp(-wo_ * dist_(j, l));
                            }

                            break;
                        case VERTEX_TYPE::GOAL:
                            d_ = ((gManager_.getGraph()->getVertex(j)->position_
                                   - gManager_.getGraph()->getVertex(l)->position_).dot(
                                      Eigen::Vector3d(Kinematics.J[j].data.block(0, i, 3, 1)))) / dist_(j, l);
                            J(cnt, i) = gManager_.getGraph()->getVertex(l)->w_ * d_;
                            break;
                        default:
                            break;
                        }
                    }
                    cnt++;
                }
            }
        }
    }

    void DMeshROS::updateGraphFromKS()
    {
        int M = Kinematics.Phi.rows();
        Eigen::VectorXd EffPhi(M*3);
        for(int i=0;i<M;i++)
        {
            EffPhi(i*3) = Kinematics.Phi(i).p[0];
            EffPhi(i*3+1) = Kinematics.Phi(i).p[1];
            EffPhi(i*3+2) = Kinematics.Phi(i).p[2];
        }
        gManager_.getGraph()->updateLinksRef(EffPhi);
    }

    void DMeshROS::updateGraphFromExternal(const std::string & name,
                                           const Eigen::Vector3d & pose)
    {
        if (!gManager_.getGraph()->updateLink(name, pose))
        {
            throw_named("Can't update the link!");
        }
    }

    void DMeshROS::updateGraphFromTF()
    {
        Eigen::VectorXd tmp = Eigen::VectorXd::Zero(robot_size_ * 3);
        for (int i = 1; i < robot_size_ - 1; i++)
        {
            try
            {
                listener_.lookupTransform("/base", "/" + links_[i], ros::Time(0), transform_);
            } catch (tf::TransformException &ex)
            {

                ros::Duration(1.0).sleep();
                throw_named(ex.what());
            }
            tmp(3 * i) = transform_.getOrigin().x();
            tmp(3 * i + 1) = transform_.getOrigin().y();
            tmp(3 * i + 2) = transform_.getOrigin().z();
        }
        tmp(3 * (robot_size_ - 1)) = 0.3;
        tmp(3 * (robot_size_ - 1) + 1) = 5;
        tmp(3 * (robot_size_ - 1) + 2) = 0.2;

        if (!gManager_.getGraph()->updateLinksRef(tmp))
        {
            throw_named("Can't update link references!");
        }
    }

    void DMeshROS::updateExternal(const exotica::MeshVertex & ext)
    {
        if (!gManager_.getGraph()->updateExternal(ext))
        {
            throw_named("Update "<<ext.name<<" failed");
        }
    }

  void DMeshROS::updateExternal(const exotica::MeshVertexArray & ext)
  {
    for (int i = 0; i < ext.vertices.size(); i++)
    {
      updateExternal(ext.vertices[i]);
    }
  }

  void DMeshROS::removeVertex(const std::string & name)
  {
    if (!gManager_.getGraph()->removeVertex(name))
    {
      ROS_ERROR_STREAM("Remove "<<name<<" failed");
    }
  }

  bool DMeshROS::hasActiveObstacle()
  {
    return gManager_.getGraph()->hasActiveObstacle();
  }
}

