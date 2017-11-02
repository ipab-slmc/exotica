/*
 *      Author: Wolfgang Merkt
 *
 * Copyright (c) 2017, Wolfgang Merkt
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

#ifndef SMOOTH_COLLISION_DISTANCE_H
#define SMOOTH_COLLISION_DISTANCE_H

#include <exotica/TaskMap.h>
#include <task_map/SmoothCollisionDistanceInitializer.h>
#include <algorithm>
#include <cmath>

namespace exotica
{
class SmoothCollisionDistance
    : public TaskMap,
      public Instantiable<SmoothCollisionDistanceInitializer>
{
public:
    SmoothCollisionDistance();
    virtual ~SmoothCollisionDistance() {}
    virtual void Instantiate(SmoothCollisionDistanceInitializer& init);

    virtual void assignScene(Scene_ptr scene);

    void Initialize();

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi,
                        Eigen::MatrixXdRef J);

    virtual int taskSpaceDim();

private:
    std::vector<std::string> robotLinks_;
    double robot_margin_ = 0.0;
    double world_margin_ = 0.0;
    bool linear_ = false;
    bool check_self_collision_ = true;

    unsigned int dim_ = 1;
    Scene_ptr scene_;
    CollisionScene_ptr cscene_;
    SmoothCollisionDistanceInitializer init_;
};

typedef std::shared_ptr<exotica::SmoothCollisionDistance>
    SmoothCollisionDistance_ptr;
}
#endif
