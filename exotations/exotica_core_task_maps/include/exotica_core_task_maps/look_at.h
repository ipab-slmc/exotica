/*
 *      Author: Christopher E. Mower
 *
 * Copyright (c) 2018, University of Edinburgh
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

#ifndef EXOTICA_CORE_TASK_MAPS_LOOKAT_H_
#define EXOTICA_CORE_TASK_MAPS_LOOKAT_H_

#include <exotica/TaskMap.h>
#include <exotica_core_task_maps/LookAtInitializer.h>

namespace exotica
{
/** 
 * \class LookAt
 * 
 * \ingroup TaskMap
 * 
 * \brief Points end-effector to look at a given target.
 *
 * Looks at a target point by penalizing the vector which defines the orthogonal projection onto a defined line in the end-effector frame.
 * 
 * The task map relies on defining three frames for each target to be looked at:
 *   - (0) the EffPoint in the end-effector frame (use in task map)
 *   - (1) the LookAtTarget in the end-effector frame (use in task map)
 *   - (2) the LookAtTarget in the world frame (use by user to easily return target in world coordinates - can be retrieved via get_look_at_target_in_world)
 * 
 * \image html taskmap_lookat.png "LookAt task map." width=500px
 * 
 * Given the point \f$p\f$ (the point to look at) defined in the end-effector space the task map is expressed by 
 * \f[
 *   \Phi = \vec{d}
 * \f]
 * where \f$\vec{d}:=p - a\f$ where \f$a\f$ is the orthogonal projection onto the line \f$L\f$ given by 
 * \f[
 *   L:=\{\alpha c : \alpha\in\mathbb{R}\}
 * \f]
 * and \f$c\f$ is some fixed point in the end-effector frame.
 * 
 * The LookAt task map can handle a goal for each end-effector. Three frames must be defined in the .xml for every goal. 
 */
class LookAt : public TaskMap, public Instantiable<LookAtInitializer>
{
public:
    LookAt();
    virtual ~LookAt();

    void Instantiate(LookAtInitializer& init) override;
    void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) override;
    void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J) override;
    int taskSpaceDim() override;

    Eigen::Vector3d get_look_at_target_in_world(const int& i);

private:
    int n_end_effs_;  ///< Number of end-effectors.
    int n_;           ///< Dimension of the task space.
};

}  // namespace exotica

#endif /* EXOTICA_CORE_TASK_MAPS_LOOKAT_H_ */
