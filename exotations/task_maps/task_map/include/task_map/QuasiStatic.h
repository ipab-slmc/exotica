/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2017, University Of Edinburgh
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

#ifndef QuasiStatic_H_
#define QuasiStatic_H_

#include <exotica/KinematicTree.h>
#include <exotica/TaskMap.h>
#include <task_map/QuasiStaticInitializer.h>
#include <visualization_msgs/MarkerArray.h>

namespace exotica
{
class QuasiStatic : public TaskMap, public Instantiable<QuasiStaticInitializer>
{
public:
    QuasiStatic();
    virtual ~QuasiStatic();

    virtual void Instantiate(QuasiStaticInitializer& init);

    virtual void assignScene(Scene_ptr scene);

    void Initialize();

    void InitDebug();

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J);

    virtual int taskSpaceDim();

private:
    QuasiStaticInitializer init_;
    Scene_ptr scene_;
    visualization_msgs::MarkerArray debug_msg;
    ros::Publisher debug_pub;
};
}

#endif /* QuasiStatic_H_ */
