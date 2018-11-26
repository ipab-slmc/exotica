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

#ifndef EXOTICA_TASK_MAP_H
#define EXOTICA_TASK_MAP_H

#include <exotica/Factory.h>  //!< The Factory template
#include <exotica/Object.h>   //!< The EXOTica base class
#include <exotica/Property.h>
#include <exotica/Scene.h>
#include <exotica/Server.h>
#include <exotica/TaskSpaceVector.h>

#include <Eigen/Dense>  //!< Generally dense manipulations should be enough
#include <string>

/**
 * \brief Convenience registrar for the TaskMap Type
 */
#define REGISTER_TASKMAP_TYPE(TYPE, DERIV) EXOTICA_REGISTER(exotica::TaskMap, TYPE, DERIV)

namespace exotica
{
class PlanningProblem;

class TaskMap : public Object, Uncopyable, public virtual InstantiableBase
{
public:
    /**
       * \brief Default Constructor
       */
    TaskMap();
    virtual ~TaskMap() {}
    virtual void InstantiateBase(const Initializer& init);

    virtual void assignScene(Scene_ptr scene) {}
    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi) = 0;
    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J);
    virtual void update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J, HessianRef H);
    virtual int taskSpaceDim() = 0;

    virtual int taskSpaceJacobianDim() { return taskSpaceDim(); }
    virtual void preupdate() {}
    virtual std::vector<TaskVectorEntry> getLieGroupIndices() { return std::vector<TaskVectorEntry>(); }
    virtual std::string print(std::string prepend);

    std::vector<KinematicFrameRequest> GetFrames();

    virtual void debug() {}
    std::vector<KinematicSolution> Kinematics;
    int Id;
    int Start;
    int Length;
    int StartJ;
    int LengthJ;
    bool isUsed;

protected:
    std::vector<KinematicFrameRequest> Frames;
};

//!< Typedefines for some common functionality
typedef Factory<TaskMap> TaskMap_fac;                    //!< Task Map Factory
typedef std::shared_ptr<TaskMap> TaskMap_ptr;            //!< Task Map smart pointer
typedef std::map<std::string, TaskMap_ptr> TaskMap_map;  //!< The mapping by name of TaskMaps
typedef std::vector<TaskMap_ptr> TaskMap_vec;
}
#endif
