/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2018, University Of Edinburgh
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

#include "task_map/CollisionFallback.h"

REGISTER_TASKMAP_TYPE("CollisionFallback", exotica::CollisionFallback);

namespace exotica
{
CollisionFallback::CollisionFallback()
{
}

void CollisionFallback::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != 1) throw_named("Wrong size of phi!");
    if (!scene_->alwaysUpdatesCollisionScene()) cscene_->updateCollisionObjectTransforms();
    phi(0) = cscene_->isStateValid(init_.SelfCollision, init_.SafeDistance) ? 0.0 : 1.0;
}

void CollisionFallback::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != 1) throw_named("Wrong size of phi!");
    if (!scene_->alwaysUpdatesCollisionScene()) cscene_->updateCollisionObjectTransforms();
    double t = scene_->getLastT();
    if(cscene_->isStateValid(init_.SelfCollision, init_.SafeDistance))
    {
        lastValid[t] = x;
        phi(0) = 0.0;
    }
    else
    {
        if( lastValid.find(t) == lastValid.end()) throw_named("No collision free initial trajectory/pose provided!");
        phi(0) = 1;
    }
    J = (x - lastValid[t]).transpose();
}

void CollisionFallback::Instantiate(CollisionFallbackInitializer& init)
{
    init_ = init;
}

void CollisionFallback::assignScene(Scene_ptr scene)
{
    scene_ = scene;
    Initialize();
}

void CollisionFallback::Initialize()
{
    cscene_ = scene_->getCollisionScene();
}

int CollisionFallback::taskSpaceDim()
{
    return 1;
}
}
