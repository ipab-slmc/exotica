/*
 * RRTConnect.cpp
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#include <ompl_solver/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

REGISTER_MOTIONSOLVER_TYPE("RRTConnect", exotica::RRTConnect)

namespace exotica {
RRTConnect::RRTConnect() {
}

RRTConnect::~RRTConnect() {

}

void RRTConnect::Instantiate(RRTConnectInitializer& init) {
	init_ = OMPLsolverInitializer(init);
	algorithm_="Exotica_RRTConnect";
	planner_allocator_ = boost::bind(&allocatePlanner<ompl::geometric::RRTConnect>, _1, _2);
}
}
