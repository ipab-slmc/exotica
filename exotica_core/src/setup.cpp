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

#include <exotica_core/scene.h>
#include <exotica_core/setup.h>
#include <type_traits>

namespace exotica
{
Setup_ptr Setup::singleton_initialiser_ = nullptr;

void Setup::printSupportedClasses()
{
    HIGHLIGHT("Registered solvers:");
    std::vector<std::string> solvers = Instance()->solvers_.getDeclaredClasses();
    for (std::string s : solvers)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered problems:");
    std::vector<std::string> problems = Instance()->problems_.getDeclaredClasses();
    for (std::string s : problems)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered task maps:");
    std::vector<std::string> maps = Instance()->maps_.getDeclaredClasses();
    for (std::string s : maps)
    {
        HIGHLIGHT(" '" << s << "'");
    }
    HIGHLIGHT("Registered collision scenes:");
    std::vector<std::string> scenes = Instance()->scenes_.getDeclaredClasses();
    for (std::string s : scenes)
    {
        HIGHLIGHT(" '" << s << "'");
    }
}

void appendInit(std::shared_ptr<InstantiableBase> it, std::vector<Initializer>& ret)
{
    std::vector<Initializer> temps = it->getAllTemplates();
    for (Initializer& i : temps)
    {
        bool found = false;
        for (Initializer& j : ret)
        {
            if (i.name == j.name)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            ret.push_back(i);
        }
    }
}

std::vector<Initializer> Setup::getInitializers()
{
    std::vector<Initializer> ret = Scene().getAllTemplates();
    std::vector<std::string> solvers = Instance()->solvers_.getDeclaredClasses();
    for (std::string s : solvers)
    {
        appendInit(std::static_pointer_cast<InstantiableBase>(createSolver(s, false)), ret);
    }
    std::vector<std::string> maps = Instance()->maps_.getDeclaredClasses();
    for (std::string s : maps)
    {
        appendInit(std::static_pointer_cast<InstantiableBase>(createMap(s, false)), ret);
    }
    return ret;
}

std::vector<std::string> Setup::getSolvers() { return Instance()->solvers_.getDeclaredClasses(); }
std::vector<std::string> Setup::getProblems() { return Instance()->problems_.getDeclaredClasses(); }
std::vector<std::string> Setup::getMaps() { return Instance()->maps_.getDeclaredClasses(); }
std::vector<std::string> Setup::getCollisionScenes() { return Instance()->scenes_.getDeclaredClasses(); }
Setup::Setup() : solvers_("exotica_core", "exotica::MotionSolver"), maps_("exotica_core", "exotica::TaskMap"), problems_(PlanningProblem_fac::Instance()), scenes_("exotica_core", "exotica::CollisionScene")
{
}
}
