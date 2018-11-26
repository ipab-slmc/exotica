/*
 *      Author: Michael Camilleri
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

#include <exotica/TaskMap.h>

#include <exotica/FrameInitializer.h>
#include <exotica/TaskMapInitializer.h>

namespace exotica
{
TaskMap::TaskMap() = default;
TaskMap::~TaskMap() = default;

void TaskMap::assignScene(Scene_ptr scene)
{
    scene_ = scene;
}

std::string TaskMap::print(std::string prepend)
{
    std::string ret = Object::print(prepend);
    return ret;
}

void TaskMap::InstantiateBase(const Initializer& init)
{
    Object::InstatiateObject(init);
    TaskMapInitializer MapInitializer(init);
    isUsed = true;

    Frames.clear();

    for (Initializer& eff : MapInitializer.EndEffector)
    {
        FrameInitializer frame(eff);
        Frames.push_back(KinematicFrameRequest(frame.Link, getFrame(frame.LinkOffset), frame.Base, getFrame(frame.BaseOffset)));
    }
}

std::vector<KinematicFrameRequest> TaskMap::GetFrames()
{
    return Frames;
}

void TaskMap::taskSpaceDim(int& task_dim)
{
    task_dim = taskSpaceDim();
}

void TaskMap::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (J.rows() != taskSpaceDim() && J.cols() != x.rows())
        throw_named("Jacobian dimension mismatch!");

    // Store original x (needs to be reset later).
    Eigen::VectorXd x_original(x);
    update(x_original, phi);
    Eigen::VectorXd phi_original(phi);

    // Backward finite differencing.
    const double h = 1e-6;
    Eigen::VectorXd x_tmp;
    for (int i = 0; i < taskSpaceDim(); i++)
    {
        x_tmp = x;
        x_tmp(i) -= h;
        update(x_tmp, phi);
        J.row(i) = (1 / h) * (phi_original - phi);
    }

    // Finally, reset with original value again.
    update(x_original, phi);
}

void TaskMap::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J, HessianRef H)
{
    update(x, phi, J);
    H.resize(taskSpaceDim());
    for (int i = 0; i < taskSpaceDim(); i++)
    {
        H(i) = J.row(i).transpose() * J.row(i);
    }
}
}
