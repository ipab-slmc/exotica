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

#ifndef INCLUDE_OMPL_OMPL_SOLVER_H_
#define INCLUDE_OMPL_OMPL_SOLVER_H_

#include <exotica/MotionSolver.h>
#include <ompl_solver/ompl_exo.h>

typedef boost::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &si, const std::string &name)> ConfiguredPlannerAllocator;

#ifdef ROS_KINETIC
template <class T, class T1>
std::shared_ptr<T> ompl_cast(std::shared_ptr<T1> ptr)
{
    return std::static_pointer_cast<T>(ptr);
}
template <class T>
using ompl_ptr = std::shared_ptr<T>;
#else
template <class T, class T1>
boost::shared_ptr<T> ompl_cast(boost::shared_ptr<T1> ptr)
{
    return boost::static_pointer_cast<T>(ptr);
}
template <class T>
using ompl_ptr = boost::shared_ptr<T>;
#endif

namespace exotica
{
class OMPLsolver : public MotionSolver
{
public:
    OMPLsolver();

    virtual ~OMPLsolver();

    virtual void Solve(Eigen::MatrixXd &solution);
    virtual void specifyProblem(PlanningProblem_ptr pointer);

    int getRandomSeed();

protected:
    template <typename T>
    static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr &si, const std::string &new_name)
    {
        ompl::base::PlannerPtr planner(new T(si));
        if (!new_name.empty())
            planner->setName(new_name);
        return planner;
    }

    void setGoalState(const Eigen::VectorXd &qT, const double eps = 0);
    void preSolve();
    void postSolve();
    void getPath(Eigen::MatrixXd &traj, ompl::base::PlannerTerminationCondition &ptc);
    OMPLsolverInitializer init_;
    SamplingProblem_ptr prob_;
    ompl::geometric::SimpleSetupPtr ompl_simple_setup_;
    ompl::base::StateSpacePtr state_space_;
    ConfiguredPlannerAllocator planner_allocator_;
    std::string algorithm_;
    bool multiQuery;
};
}

#endif /* INCLUDE_OMPL_OMPL_SOLVER_H_ */
