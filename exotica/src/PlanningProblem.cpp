/*
 *      Author: Michael Camilleri
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

#include <exotica/PlanningProblem.h>
#include <exotica/PlanningProblemInitializer.h>
#include <exotica/Setup.h>

namespace exotica
{
PlanningProblem::PlanningProblem() : Flags(KIN_FK), N(0)
{
}

std::string PlanningProblem::print(std::string prepend)
{
    std::string ret = Object::print(prepend);
    ret += "\n" + prepend + "  Task definitions:";
    for (auto& it : TaskMaps)
        ret += "\n" + it.second->print(prepend + "    ");
    return ret;
}

Eigen::VectorXd PlanningProblem::applyStartState()
{
    scene_->setModelState(startState, tStart);
    return scene_->getControlledState();
}

void PlanningProblem::preupdate()
{
    for (auto& it : TaskMaps) it.second->preupdate();
}

void PlanningProblem::setStartState(Eigen::VectorXdRefConst x)
{
    if (x.rows() == scene_->getSolver().getNumModelJoints())
    {
        startState = x;
    }
    else if (x.rows() == scene_->getSolver().getNumControlledJoints())
    {
        std::vector<std::string> jointNames = scene_->getJointNames();
        std::vector<std::string> modelNames = scene_->getModelJointNames();
        for (int i = 0; i < jointNames.size(); i++)
        {
            for (int j = 0; j < modelNames.size(); j++)
            {
                if (jointNames[i] == modelNames[j]) startState[j] = x(i);
            }
        }
    }
    else
    {
        throw_named("Wrong start state vector size, expected " << scene_->getSolver().getNumModelJoints() << ", got " << x.rows());
    }
}

Eigen::VectorXd PlanningProblem::getStartState()
{
    return startState;
}

void PlanningProblem::setStartTime(double t)
{
    tStart = t;
}

double PlanningProblem::getStartTime()
{
    return tStart;
}

void PlanningProblem::InstantiateBase(const Initializer& init_)
{
    Object::InstatiateObject(init_);
    PlanningProblemInitializer init(init_);

    TaskMaps.clear();
    Tasks.clear();

    // Create the scene
    scene_.reset(new Scene());
    scene_->InstantiateInternal(SceneInitializer(init.PlanningScene));
    startState = Eigen::VectorXd::Zero(scene_->getModelJointNames().size());
    N = scene_->getSolver().getNumControlledJoints();

    if (init.StartState.rows() > 0)
    {
        setStartState(init.StartState);
    }
    if (init.StartTime < 0)
        throw_named("Invalid start time " << init.StartTime) else tStart = init.StartTime;

    KinematicsRequest Request;
    Request.Flags = Flags;

    // Create the maps
    int id = 0;
    for (const Initializer& MapInitializer : init.Maps)
    {
        TaskMap_ptr NewMap = Setup::createMap(MapInitializer);
        NewMap->assignScene(scene_);
        NewMap->ns_ = ns_ + "/" + NewMap->getObjectName();
        if (TaskMaps.find(NewMap->getObjectName()) != TaskMaps.end())
        {
            throw_named("Map '" + NewMap->getObjectName() + "' already exists!");
        }
        std::vector<KinematicFrameRequest> frames = NewMap->GetFrames();

        NewMap->Kinematics = KinematicSolution(id, frames.size());
        id += frames.size();

        Request.Frames.insert(Request.Frames.end(), frames.begin(), frames.end());
        TaskMaps[NewMap->getObjectName()] = NewMap;
        Tasks.push_back(NewMap);
    }

    std::shared_ptr<KinematicResponse> Response = scene_->RequestKinematics(Request);
    id = 0;
    int idJ = 0;
    for (int i = 0; i < Tasks.size(); i++)
    {
        Tasks[i]->Kinematics.Create(Response);
        Tasks[i]->Id = i;
        Tasks[i]->Start = id;
        Tasks[i]->Length = Tasks[i]->taskSpaceDim();
        Tasks[i]->StartJ = idJ;
        Tasks[i]->LengthJ = Tasks[i]->taskSpaceJacobianDim();
        id += Tasks[i]->Length;
        idJ += Tasks[i]->LengthJ;
    }

    if (init.Maps.size() == 0)
    {
        HIGHLIGHT("No maps were defined!");
    }
}

TaskMap_map& PlanningProblem::getTaskMaps()
{
    return TaskMaps;
}

TaskMap_vec& PlanningProblem::getTasks()
{
    return Tasks;
}

Scene_ptr PlanningProblem::getScene()
{
    return scene_;
}

std::vector<double> PlanningProblem::getCostEvolution()
{
    // Find first NAN and resize vector
    size_t position = 0;
    for (position = 0; position < costEvolution.size(); position++)
        if (std::isnan(costEvolution[position])) break;
    costEvolution.resize(position);
    return costEvolution;
}

double PlanningProblem::getCostEvolution(int index)
{
    if (index > -1 && index < costEvolution.size())
    {
        return costEvolution[index];
    }
    else if (index == -1)
    {
        return costEvolution[costEvolution.size() - 1];
    }
    else
    {
        throw_pretty("Out of range");
    }
}

void PlanningProblem::resetCostEvolution(unsigned int size)
{
    costEvolution.resize(size);
    costEvolution.assign(size, std::numeric_limits<double>::quiet_NaN());
}

void PlanningProblem::setCostEvolution(int index, double value)
{
    if (index > -1 && index < costEvolution.size())
    {
        costEvolution[index] = value;
    }
    else if (index == -1)
    {
        costEvolution[costEvolution.size() - 1] = value;
    }
    else
    {
        throw_pretty("Out of range: " << index << " where length=" << costEvolution.size());
    }
}
}
