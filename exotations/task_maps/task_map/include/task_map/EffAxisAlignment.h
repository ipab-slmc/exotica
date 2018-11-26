/*
 *      Author: Wolfgang Merkt
 * 
 * Copyright (c) 2018, Wolfgang Merkt
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

#ifndef EXOTICA_TASKMAP_EFF_AXIS_ALIGNMENT_H
#define EXOTICA_TASKMAP_EFF_AXIS_ALIGNMENT_H

#include <exotica/TaskMap.h>
#include <task_map/EffAxisAlignmentInitializer.h>
#include <task_map/FrameWithAxisAndDirectionInitializer.h>

namespace exotica  //!< Since this is part of the core library, it will be within the same namespace
{
class EffAxisAlignment : public TaskMap, public Instantiable<EffAxisAlignmentInitializer>
{
public:
    /**
       * \brief Default constructor
       */
    EffAxisAlignment();

    virtual void Instantiate(EffAxisAlignmentInitializer& init);

    virtual void assignScene(Scene_ptr scene);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi);

    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J);

    virtual int taskSpaceDim();

    void setDirection(const std::string& frame_name, const Eigen::Vector3d& dir_in);
    Eigen::Vector3d getDirection(const std::string& frame_name);

    void setAxis(const std::string& frame_name, const Eigen::Vector3d& axis_in);
    Eigen::Vector3d getAxis(const std::string& frame_name);

private:
    EffAxisAlignmentInitializer init_;
    unsigned int NumberOfFrames;
    unsigned int N;
    void Initialize();

    Eigen::Matrix3Xd axis_, dir_;
    Eigen::Vector3d linkPositionInBase_, linkAxisPositionInBase_;
};

typedef std::shared_ptr<EffAxisAlignment> EffAxisAlignment_ptr;  //!< Task Map smart pointer
}

#endif
