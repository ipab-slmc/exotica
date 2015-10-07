/*
 * common.h
 *
 *  Created on: 24 Jun 2014
 *      Author: s0972326
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/StateStorage.h>

namespace exotica
{

  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  namespace ot = ompl::tools;

  typedef boost::function<
      ob::PlannerPtr(const ompl::base::SpaceInformationPtr &si,
          const std::string &name)> ConfiguredPlannerAllocator;

}

#endif /* COMMON_H_ */
