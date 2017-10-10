/*
 * ompl_native_solvers.cpp
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
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

void RRT::Instantiate(OMPLsolverInitializer& init)
{
    init_ = init;
    algorithm_ = "Exotica_RRT";
    planner_allocator_ = boost::bind(
        &allocatePlanner<ompl::geometric::RRT>, _1, _2);
}

RRTConnect::RRTConnect()
{
}

void RRTConnect::Instantiate(OMPLsolverInitializer& init)
{
    init_ = init;
    algorithm_ = "Exotica_RRTConnect";
    planner_allocator_ = boost::bind(
        &allocatePlanner<ompl::geometric::RRTConnect>, _1, _2);
}

PRM::PRM()
{
}

void PRM::Instantiate(OMPLsolverInitializer& init)
{
    init_ = init;
    algorithm_ = "Exotica_PRM";
    planner_allocator_ = boost::bind(
        &allocatePlanner<ompl::geometric::PRM>, _1, _2);
}
}
