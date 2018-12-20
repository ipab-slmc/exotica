/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University of Edinburgh
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

#include "exotica_core_task_maps/interaction_mesh.h"

REGISTER_TASKMAP_TYPE("IMesh", exotica::IMesh);

namespace exotica
{
IMesh::IMesh() = default;
IMesh::~IMesh() = default;

void IMesh::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    int M = eff_size_;

    if (phi.rows() != M * 3) throw_named("Wrong size of phi!");

    Eigen::VectorXd eff_Phi;
    for (int i = 0; i < M; i++)
    {
        eff_Phi(i * 3) = Kinematics[0].Phi(i).p[0];
        eff_Phi(i * 3 + 1) = Kinematics[0].Phi(i).p[1];
        eff_Phi(i * 3 + 2) = Kinematics[0].Phi(i).p[2];
    }
    phi = compute_laplace(eff_Phi, weights_);

    if (debug_) debug(phi);
}

void IMesh::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    int M = eff_size_;
    int N = Kinematics[0].J[0].data.cols();

    if (phi.rows() != M * 3) throw_named("Wrong size of phi!");
    if (J.rows() != M * 3 || J.cols() != N) throw_named("Wrong size of J! " << N);

    Eigen::MatrixXd dist;
    Eigen::VectorXd wsum;
    Eigen::VectorXd eff_Phi(M * 3);
    for (int i = 0; i < M; i++)
    {
        eff_Phi(i * 3) = Kinematics[0].Phi(i).p[0];
        eff_Phi(i * 3 + 1) = Kinematics[0].Phi(i).p[1];
        eff_Phi(i * 3 + 2) = Kinematics[0].Phi(i).p[2];
    }
    phi = compute_laplace(eff_Phi, weights_, &dist, &wsum);

    double A, _A, Sk, Sl, w, _w;
    int i, j, k, l;
    Eigen::Vector3d distance, _distance = Eigen::Vector3d::Zero(3, 1);
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < M; j++)
        {
            if (j < M)
                J.block(3 * j, i, 3, 1) = Kinematics[0].J[j].data.block(0, i, 3, 1);
            for (l = 0; l < M; l++)
            {
                if (j != l)
                {
                    if (dist(j, l) > 0 && wsum(j) > 0 && weights_(j, l) > 0)
                    {
                        A = dist(j, l) * wsum(j);
                        w = weights_(j, l) / A;

                        _A = 0;
                        distance = eff_Phi.segment(j * 3, 3) - eff_Phi.segment(l * 3, 3);
                        _distance = Kinematics[0].J[j].data.block(0, i, 3, 1) - Kinematics[0].J[l].data.block(0, i, 3, 1);

                        Sl = distance.dot(_distance) / dist(j, l);
                        for (k = 0; k < M; k++)
                        {
                            if (j != k && dist(j, k) > 0 && weights_(j, k) > 0)
                            {
                                distance = eff_Phi.segment(j * 3, 3) - eff_Phi.segment(k * 3, 3);
                                _distance = Kinematics[0].J[j].data.block(0, i, 3, 1) - Kinematics[0].J[k].data.block(0, i, 3, 1);
                                Sk = distance.dot(_distance) / dist(j, k);
                                _A += weights_(j, k) * (Sl * dist(j, k) - Sk * dist(j, l)) / (dist(j, k) * dist(j, k));
                            }
                        }
                        _w = -weights_(j, l) * _A / (A * A);
                    }
                    else
                    {
                        _w = 0;
                        w = 0;
                    }
                    J.block(3 * j, i, 3, 1) -= eff_Phi.segment(l * 3, 3) * _w + Kinematics[0].J[l].data.block(0, i, 3, 1) * w;
                }
            }
        }
    }

    if (debug_) debug(phi);
}

Eigen::MatrixXd IMesh::get_weights()
{
    return weights_;
}

void IMesh::initialize_debug(std::string ref)
{
    imesh_mark_.header.frame_id = ref;
    imesh_mark_.ns = getObjectName();
    imesh_mark_pub_ = Server::advertise<visualization_msgs::Marker>(ns_ + "/InteractionMesh", 1, true);
    if (debug_) HIGHLIGHT("InteractionMesh connectivity is published on ROS topic " << imesh_mark_pub_.getTopic() << ", in reference frame " << ref);
}

void IMesh::Instantiate(IMeshInitializer& init)
{
    if (debug_)
    {
        initialize_debug(init.ReferenceFrame);
    }
    eff_size_ = Frames.size();
    weights_.setOnes(eff_size_, eff_size_);
    if (init.Weights.rows() == eff_size_ * eff_size_)
    {
        weights_.array() = init.Weights.array();
        HIGHLIGHT("Loading iMesh weights.\n"
                  << weights_);
    }
}

void IMesh::assignScene(Scene_ptr scene)
{
    scene_ = scene;
}

void IMesh::debug(Eigen::VectorXdRefConst phi)
{
    static int textid = 0;
    {
        imesh_mark_.scale.x = 0.005;
        imesh_mark_.color.a = imesh_mark_.color.r = 1;
        imesh_mark_.type = visualization_msgs::Marker::LINE_LIST;
        imesh_mark_.pose = geometry_msgs::Pose();
        imesh_mark_.ns = getObjectName();
        imesh_mark_.points.clear();
        std::vector<geometry_msgs::Point> points(eff_size_);
        for (int i = 0; i < eff_size_; i++)
        {
            points[i].x = Kinematics[0].Phi(i).p[0];
            points[i].y = Kinematics[0].Phi(i).p[1];
            points[i].z = Kinematics[0].Phi(i).p[2];
        }

        for (int i = 0; i < eff_size_; i++)
        {
            for (int j = i + 1; j < eff_size_; j++)
            {
                if (weights_(i, j) > 0.0)
                {
                    imesh_mark_.points.push_back(points[i]);
                    imesh_mark_.points.push_back(points[j]);
                }
            }
        }
        imesh_mark_.header.stamp = ros::Time::now();
        imesh_mark_pub_.publish(imesh_mark_);
    }
    {
        imesh_mark_.ns = getObjectName() + "Raw";
        imesh_mark_.points.clear();
        std::vector<geometry_msgs::Point> points(eff_size_);
        for (int i = 0; i < eff_size_; i++)
        {
            points[i].x = phi(i * 3);
            points[i].y = phi(i * 3 + 1);
            points[i].z = phi(i * 3 + 2);
        }

        for (int i = 0; i < eff_size_; i++)
        {
            for (int j = i + 1; j < eff_size_; j++)
            {
                if (weights_(i, j) > 0.0)
                {
                    imesh_mark_.points.push_back(points[i]);
                    imesh_mark_.points.push_back(points[j]);
                }
            }
        }
        imesh_mark_.header.stamp = ros::Time::now();
        imesh_mark_pub_.publish(imesh_mark_);

        imesh_mark_.points.clear();
        imesh_mark_.scale.z = 0.05;
        imesh_mark_.color.a = imesh_mark_.color.r = imesh_mark_.color.g = imesh_mark_.color.b = 1;
        imesh_mark_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        imesh_mark_.text = std::to_string(textid);
        imesh_mark_.pose.position = points[textid];
        imesh_mark_.pose.position.z += 0.05;
        imesh_mark_.ns = getObjectName() + "Id";
        imesh_mark_.header.stamp = ros::Time::now();
        imesh_mark_pub_.publish(imesh_mark_);

        textid = (textid + 1) % eff_size_;
    }
}

void IMesh::destroy_debug()
{
    imesh_mark_.points.clear();
    imesh_mark_.action = visualization_msgs::Marker::DELETE;
    imesh_mark_.header.stamp = ros::Time::now();
    imesh_mark_pub_.publish(imesh_mark_);
}

int IMesh::taskSpaceDim()
{
    return 3 * eff_size_;
}

Eigen::VectorXd IMesh::compute_laplace(Eigen::VectorXdRefConst eff_phi, Eigen::MatrixXdRefConst weights, Eigen::MatrixXd* dist_ptr, Eigen::VectorXd* wsum_ptr)
{
    int N = eff_phi.rows() / 3;
    Eigen::VectorXd Phi = Eigen::VectorXd::Zero(N * 3);
    Eigen::MatrixXd dist = Eigen::MatrixXd::Zero(N, N);
    Eigen::VectorXd wsum = Eigen::VectorXd::Zero(N);
    /** Compute distance matrix (inverse proportional) */
    for (int j = 0; j < N; j++)
    {
        for (int l = j + 1; l < N; l++)
        {
            if (!(j >= N && l >= N))
            {
                dist(j, l) = dist(l, j) = (eff_phi.segment(j * 3, 3) - eff_phi.segment(l * 3, 3)).norm();
            }
        }
    }
    /** Computer weight normaliser */
    for (int j = 0; j < N; j++)
    {
        for (int l = 0; l < N; l++)
        {
            if (dist(j, l) > 0 && j != l)
            {
                wsum(j) += weights(j, l) / dist(j, l);
            }
        }
    }
    /** Compute Laplace coordinates */
    for (int j = 0; j < N; j++)
    {
        Phi.segment(j * 3, 3) = eff_phi.segment(j * 3, 3);
        for (int l = 0; l < N; l++)
        {
            if (j != l)
            {
                if (dist(j, l) > 0 && wsum(j) > 0)
                {
                    Phi.segment(j * 3, 3) -= eff_phi.segment(l * 3, 3) * weights(j, l) / (dist(j, l) * wsum(j));
                }
            }
        }
    }
    if (dist_ptr) *dist_ptr = dist;
    if (wsum_ptr) *wsum_ptr = wsum;
    return Phi;
}

void IMesh::compute_goal_laplace(const std::vector<KDL::Frame>& nodes, Eigen::VectorXd& goal, Eigen::MatrixXdRefConst weights)
{
    int N = nodes.size();
    Eigen::VectorXd eff_phi(3 * N);
    for (int i = 0; i < N; i++)
    {
        eff_phi(i * 3) = nodes[i].p[0];
        eff_phi(i * 3 + 1) = nodes[i].p[1];
        eff_phi(i * 3 + 2) = nodes[i].p[2];
    }
    goal = compute_laplace(eff_phi, weights);
}

void IMesh::compute_goal_laplace(const Eigen::VectorXd& x, Eigen::VectorXd& goal)
{
    scene_->Update(x);
    Eigen::VectorXd eff_phi;
    for (int i = 0; i < eff_size_; i++)
    {
        eff_phi(i * 3) = Kinematics[0].Phi(i).p[0];
        eff_phi(i * 3 + 1) = Kinematics[0].Phi(i).p[1];
        eff_phi(i * 3 + 2) = Kinematics[0].Phi(i).p[2];
    }
    goal = compute_laplace(eff_phi, weights_);
}

void IMesh::set_weight(int i, int j, double weight)
{
    uint M = weights_.cols();
    if (i < 0 || i >= M || j < 0 || j >= M)
    {
        throw_named("Invalid weight element (" << i << "," << j << "). Weight matrix " << M << "x" << M);
    }
    if (weight < 0)
    {
        throw_named("Invalid weight: " << weight);
    }
    weights_(i, j) = weight;
}

void IMesh::set_weights(const Eigen::MatrixXd& weights)
{
    uint M = weights_.cols();
    if (weights.rows() != M || weights.cols() != M)
    {
        throw_named("Invalid weight matrix (" << weights.rows() << "X" << weights.cols() << "). Has to be" << M << "x" << M);
    }
    weights_ = weights;
}
}
