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

#ifndef EXOTICA_INITIALISER_H
#define EXOTICA_INITIALISER_H

#include <exotica/Property.h>
#include "exotica/Factory.h"
#include "exotica/MotionSolver.h"
#include "exotica/Object.h"
#include "exotica/PlanningProblem.h"
#include "exotica/Server.h"
#include "exotica/Tools.h"

#include <pluginlib/class_loader.h>

namespace exotica
{
class Setup : public Object, Uncopyable
{
public:
    ~Setup() noexcept
    {
    }

    static std::shared_ptr<Setup> Instance()
    {
        if (!singleton_initialiser_) singleton_initialiser_.reset(new Setup);
        return singleton_initialiser_;
    }

    static void Destroy()
    {
        Server::destroy();
        if (singleton_initialiser_) singleton_initialiser_.reset();
    }

    static void printSupportedClasses();
    static std::shared_ptr<exotica::MotionSolver> createSolver(const std::string& type, bool prepend = true) { return to_std_ptr(Instance()->solvers_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::shared_ptr<exotica::TaskMap> createMap(const std::string& type, bool prepend = true) { return to_std_ptr(Instance()->maps_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::shared_ptr<exotica::PlanningProblem> createProblem(const std::string& type, bool prepend = true) { return Instance()->problems_.createInstance((prepend ? "exotica/" : "") + type); }
    static std::shared_ptr<exotica::CollisionScene> createCollisionScene(const std::string& type, bool prepend = true) { return to_std_ptr(Instance()->scenes_.createInstance((prepend ? "exotica/" : "") + type)); }
    static std::vector<std::string> getSolvers();
    static std::vector<std::string> getProblems();
    static std::vector<std::string> getMaps();
    static std::vector<std::string> getCollisionScenes();
    static std::vector<Initializer> getInitializers();

    static std::shared_ptr<exotica::MotionSolver> createSolver(const Initializer& init)
    {
        std::shared_ptr<exotica::MotionSolver> ret = to_std_ptr(Instance()->solvers_.createInstance(init.getName()));
        ret->InstantiateInternal(init);
        return ret;
    }
    static std::shared_ptr<exotica::TaskMap> createMap(const Initializer& init)
    {
        std::shared_ptr<exotica::TaskMap> ret = to_std_ptr(Instance()->maps_.createInstance(init.getName()));
        ret->InstantiateInternal(init);
        return ret;
    }
    static std::shared_ptr<exotica::PlanningProblem> createProblem(const Initializer& init)
    {
        std::shared_ptr<exotica::PlanningProblem> ret = Instance()->problems_.createInstance(init.getName());
        ret->InstantiateInternal(init);
        return ret;
    }

private:
    /**
       * \brief Default Constructor
       *
       *        Currently, is an empty constructor definition.
       */
    Setup();
    static std::shared_ptr<Setup> singleton_initialiser_;
    ///	\brief	Make sure the singleton does not get copied
    Setup(Setup const&) = delete;
    void operator=(Setup const&) = delete;

    pluginlib::ClassLoader<exotica::MotionSolver> solvers_;
    pluginlib::ClassLoader<exotica::TaskMap> maps_;
    pluginlib::ClassLoader<exotica::CollisionScene> scenes_;
    PlanningProblem_fac problems_;
};

typedef std::shared_ptr<Setup> Setup_ptr;
}

#endif
