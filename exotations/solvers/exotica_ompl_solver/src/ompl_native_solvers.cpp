/*
 *  Created on: 10 Oct 2017
 *      Author: Yiming Yang
 *
 * Copyright (c) 2017, University of Edinburgh
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

#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <exotica_ompl_solver/ompl_native_solvers.h>

REGISTER_MOTIONSOLVER_TYPE("RRT", exotica::RRT)
REGISTER_MOTIONSOLVER_TYPE("RRTConnect", exotica::RRTConnect)
REGISTER_MOTIONSOLVER_TYPE("PRM", exotica::PRM)
REGISTER_MOTIONSOLVER_TYPE("LazyPRM", exotica::LazyPRM)
REGISTER_MOTIONSOLVER_TYPE("EST", exotica::EST)
REGISTER_MOTIONSOLVER_TYPE("KPIECE", exotica::KPIECE)
REGISTER_MOTIONSOLVER_TYPE("BKPIECE", exotica::BKPIECE)
REGISTER_MOTIONSOLVER_TYPE("RRTStar", exotica::RRTStar)
REGISTER_MOTIONSOLVER_TYPE("LBTRRT", exotica::LBTRRT)

namespace exotica
{
RRTStar::RRTStar() = default;

void RRTStar::Instantiate(RRTStarInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_RRTStar";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::RRTstar>, _1, _2);
}

LBTRRT::LBTRRT() = default;

void LBTRRT::Instantiate(LBTRRTInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_LBTRRT";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::LBTRRT>, _1, _2);
}

RRT::RRT() = default;

void RRT::Instantiate(RRTInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_RRT";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::RRT>, _1, _2);
}

RRTConnect::RRTConnect() = default;

void RRTConnect::Instantiate(RRTConnectInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_RRTConnect";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::RRTConnect>, _1, _2);
}

void RRTConnect::SetRange(double range)
{
    ompl_ptr<ompl::geometric::RRTConnect> rrtcon = ompl_cast<ompl::geometric::RRTConnect>(ompl_simple_setup_->getPlanner());
    rrtcon->setRange(range);
}

double RRTConnect::GetRange()
{
    ompl_ptr<ompl::geometric::RRTConnect> rrtcon = ompl_cast<ompl::geometric::RRTConnect>(ompl_simple_setup_->getPlanner());
    return rrtcon->getRange();
}

EST::EST() = default;

void EST::Instantiate(ESTInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_EST";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::EST>, _1, _2);
}

KPIECE::KPIECE() = default;

void KPIECE::Instantiate(KPIECEInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_KPIECE";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::KPIECE1>, _1, _2);
}

BKPIECE::BKPIECE() = default;

void BKPIECE::Instantiate(BKPIECEInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_BKPIECE";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::BKPIECE1>, _1, _2);
}

PRM::PRM() = default;

void PRM::Instantiate(PRMInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_PRM";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::PRM>, _1, _2);
    multi_query_ = init.MultiQuery;
}

void PRM::GrowRoadmap(double t)
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->growRoadmap(t);
}

void PRM::ExpandRoadmap(double t)
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->expandRoadmap(t);
}

void PRM::Clear()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    ompl_simple_setup_->getPlanner()->setProblemDefinition(ompl_simple_setup_->getProblemDefinition());
    prm->clear();
}

void PRM::ClearQuery()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->clearQuery();
}

void PRM::Setup()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->setup();
}

int PRM::EdgeCount()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    return prm->edgeCount();
}

int PRM::MilestoneCount()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    return prm->milestoneCount();
}

bool PRM::IsMultiQuery() const
{
    return multi_query_;
}

void PRM::SetMultiQuery(bool val)
{
    multi_query_ = val;
}

LazyPRM::LazyPRM() = default;

void LazyPRM::Instantiate(LazyPRMInitializer& init)
{
    init_ = static_cast<Initializer>(init);
    algorithm_ = "Exotica_LazyPRM";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::LazyPRM>, _1, _2);
    multi_query_ = init.MultiQuery;
}

void LazyPRM::Clear()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    prm->clear();
}

void LazyPRM::ClearQuery()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    prm->clearQuery();
}

void LazyPRM::Setup()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    prm->setup();
}

int LazyPRM::EdgeCount()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    return prm->edgeCount();
}

int LazyPRM::MilestoneCount()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    return prm->milestoneCount();
}

bool LazyPRM::IsMultiQuery() const
{
    return multi_query_;
}

void LazyPRM::SetMultiQuery(bool val)
{
    multi_query_ = val;
}
}
