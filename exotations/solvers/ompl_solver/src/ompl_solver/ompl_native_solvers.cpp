/*
 *  Created on: 10 Oct 2017
 *      Author: Yiming Yang
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

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl_solver/ompl_native_solvers.h>

REGISTER_MOTIONSOLVER_TYPE("RRT", exotica::RRT)
REGISTER_MOTIONSOLVER_TYPE("RRTConnect", exotica::RRTConnect)
REGISTER_MOTIONSOLVER_TYPE("PRM", exotica::PRM)

namespace exotica
{
RRT::RRT()
{
}

void RRT::Instantiate(RRTInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_RRT";
    planner_allocator_ = boost::bind(
        &allocatePlanner<ompl::geometric::RRT>, _1, _2);
}

RRTConnect::RRTConnect()
{
}

void RRTConnect::Instantiate(RRTConnectInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_RRTConnect";
    planner_allocator_ = boost::bind(
        &allocatePlanner<ompl::geometric::RRTConnect>, _1, _2);
}

PRM::PRM()
{
}

void PRM::Instantiate(PRMInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_PRM";
    planner_allocator_ = boost::bind(
        &allocatePlanner<ompl::geometric::PRM>, _1, _2);
}
}
