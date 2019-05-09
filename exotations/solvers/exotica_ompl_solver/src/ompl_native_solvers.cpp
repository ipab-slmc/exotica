//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

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

REGISTER_MOTIONSOLVER_TYPE("RRTSolver", exotica::RRTSolver)
REGISTER_MOTIONSOLVER_TYPE("RRTConnectSolver", exotica::RRTConnectSolver)
REGISTER_MOTIONSOLVER_TYPE("PRMSolver", exotica::PRMSolver)
REGISTER_MOTIONSOLVER_TYPE("LazyPRMSolver", exotica::LazyPRMSolver)
REGISTER_MOTIONSOLVER_TYPE("ESTSolver", exotica::ESTSolver)
REGISTER_MOTIONSOLVER_TYPE("KPIECESolver", exotica::KPIECESolver)
REGISTER_MOTIONSOLVER_TYPE("BKPIECESolver", exotica::BKPIECESolver)
REGISTER_MOTIONSOLVER_TYPE("RRTStarSolver", exotica::RRTStarSolver)
REGISTER_MOTIONSOLVER_TYPE("LBTRRTSolver", exotica::LBTRRTSolver)

namespace exotica
{
RRTStarSolver::RRTStarSolver() = default;

void RRTStarSolver::Instantiate(const RRTStarSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(RRTStarSolverInitializer(init));
    algorithm_ = "Exotica_RRTStar";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::RRTstar>, _1, _2);
}

LBTRRTSolver::LBTRRTSolver() = default;

void LBTRRTSolver::Instantiate(const LBTRRTSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(LBTRRTSolverInitializer(init));
    algorithm_ = "Exotica_LBTRRT";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::LBTRRT>, _1, _2);
}

RRTSolver::RRTSolver() = default;

void RRTSolver::Instantiate(const RRTSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(RRTSolverInitializer(init));
    algorithm_ = "Exotica_RRT";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::RRT>, _1, _2);
}

RRTConnectSolver::RRTConnectSolver() = default;

void RRTConnectSolver::Instantiate(const RRTConnectSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(RRTConnectSolverInitializer(init));
    algorithm_ = "Exotica_RRTConnect";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::RRTConnect>, _1, _2);
}

void RRTConnectSolver::SetRange(double range)
{
    ompl_ptr<ompl::geometric::RRTConnect> rrtcon = ompl_cast<ompl::geometric::RRTConnect>(ompl_simple_setup_->getPlanner());
    rrtcon->setRange(range);
}

double RRTConnectSolver::GetRange()
{
    ompl_ptr<ompl::geometric::RRTConnect> rrtcon = ompl_cast<ompl::geometric::RRTConnect>(ompl_simple_setup_->getPlanner());
    return rrtcon->getRange();
}

ESTSolver::ESTSolver() = default;

void ESTSolver::Instantiate(const ESTSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(ESTSolverInitializer(init));
    algorithm_ = "Exotica_EST";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::EST>, _1, _2);
}

KPIECESolver::KPIECESolver() = default;

void KPIECESolver::Instantiate(const KPIECESolverInitializer &init)
{
    init_ = OMPLSolverInitializer(KPIECESolverInitializer(init));
    algorithm_ = "Exotica_KPIECE";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::KPIECE1>, _1, _2);
}

BKPIECESolver::BKPIECESolver() = default;

void BKPIECESolver::Instantiate(const BKPIECESolverInitializer &init)
{
    init_ = OMPLSolverInitializer(BKPIECESolverInitializer(init));
    algorithm_ = "Exotica_BKPIECE";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::BKPIECE1>, _1, _2);
}

PRMSolver::PRMSolver() = default;

void PRMSolver::Instantiate(const PRMSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(PRMSolverInitializer(init));
    algorithm_ = "Exotica_PRM";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::PRM>, _1, _2);
    multi_query_ = init.MultiQuery;
}

void PRMSolver::GrowRoadmap(double t)
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->growRoadmap(t);
}

void PRMSolver::ExpandRoadmap(double t)
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->expandRoadmap(t);
}

void PRMSolver::Clear()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    ompl_simple_setup_->getPlanner()->setProblemDefinition(ompl_simple_setup_->getProblemDefinition());
    prm->clear();
}

void PRMSolver::ClearQuery()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->clearQuery();
}

void PRMSolver::Setup()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    prm->setup();
}

int PRMSolver::EdgeCount()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    return prm->edgeCount();
}

int PRMSolver::MilestoneCount()
{
    ompl_ptr<ompl::geometric::PRM> prm = ompl_cast<ompl::geometric::PRM>(ompl_simple_setup_->getPlanner());
    return prm->milestoneCount();
}

bool PRMSolver::IsMultiQuery() const
{
    return multi_query_;
}

void PRMSolver::SetMultiQuery(bool val)
{
    multi_query_ = val;
}

LazyPRMSolver::LazyPRMSolver() = default;

void LazyPRMSolver::Instantiate(const LazyPRMSolverInitializer &init)
{
    init_ = OMPLSolverInitializer(LazyPRMSolverInitializer(init));
    algorithm_ = "Exotica_LazyPRM";
    planner_allocator_ = boost::bind(&AllocatePlanner<ompl::geometric::LazyPRM>, _1, _2);
    multi_query_ = init.MultiQuery;
}

void LazyPRMSolver::Clear()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    prm->clear();
}

void LazyPRMSolver::ClearQuery()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    prm->clearQuery();
}

void LazyPRMSolver::Setup()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    prm->setup();
}

int LazyPRMSolver::EdgeCount()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    return prm->edgeCount();
}

int LazyPRMSolver::MilestoneCount()
{
    ompl_ptr<ompl::geometric::LazyPRM> prm = ompl_cast<ompl::geometric::LazyPRM>(ompl_simple_setup_->getPlanner());
    return prm->milestoneCount();
}

bool LazyPRMSolver::IsMultiQuery() const
{
    return multi_query_;
}

void LazyPRMSolver::SetMultiQuery(bool val)
{
    multi_query_ = val;
}
}
