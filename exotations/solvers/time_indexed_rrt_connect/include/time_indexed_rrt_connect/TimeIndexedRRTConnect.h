/*
 *  Created on: 7 Nov 2017
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

#ifndef TIME_INDEXED_RRT_CONNECT_TIMEINDEXEDRRTCONNECT_H_
#define TIME_INDEXED_RRT_CONNECT_TIMEINDEXEDRRTCONNECT_H_

#include <exotica/Problems/TimeIndexedSamplingProblem.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <time_indexed_rrt_connect/TimeIndexedRRTConnectInitializer.h>

#include <exotica/MotionSolver.h>

namespace exotica
{
class OMPLTimeIndexedRNStateSpace : public ompl::base::CompoundStateSpace
{
public:
    class StateType : public ompl::base::CompoundStateSpace::StateType
    {
    public:
        StateType() : CompoundStateSpace::StateType()
        {
        }

        const ompl::base::RealVectorStateSpace::StateType &getRNSpace() const
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(0);
        }

        ompl::base::RealVectorStateSpace::StateType &getRNSpace()
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(0);
        }

        const ompl::base::TimeStateSpace::StateType &getTime() const
        {
            return *as<ompl::base::TimeStateSpace::StateType>(1);
        }

        ompl::base::TimeStateSpace::StateType &getTime()
        {
            return *as<ompl::base::TimeStateSpace::StateType>(1);
        }
    };
    OMPLTimeIndexedRNStateSpace(TimeIndexedSamplingProblem_ptr &prob, TimeIndexedRRTConnectInitializer init);

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
    void ExoticaToOMPLState(const Eigen::VectorXd &q, const double &t, ompl::base::State *state) const;
    void OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q, double &t) const;
    void stateDebug(const Eigen::VectorXd &q) const;

protected:
    TimeIndexedSamplingProblem_ptr prob_;
};

class OMPLTimeIndexedStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    OMPLTimeIndexedStateValidityChecker(const ompl::base::SpaceInformationPtr &si, const TimeIndexedSamplingProblem_ptr &prob);

    virtual bool isValid(const ompl::base::State *state) const;

    virtual bool isValid(const ompl::base::State *state, double &dist) const;

protected:
    TimeIndexedSamplingProblem_ptr prob_;
};

typedef boost::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr &si, const std::string &name)> ConfiguredPlannerAllocator;

class TimeIndexedRRTConnect : public MotionSolver, Instantiable<TimeIndexedRRTConnectInitializer>
{
public:
    TimeIndexedRRTConnect();

    virtual ~TimeIndexedRRTConnect();

    virtual void Instantiate(TimeIndexedRRTConnectInitializer &init);
    virtual void Solve(Eigen::MatrixXd &solution);
    virtual void specifyProblem(PlanningProblem_ptr pointer);

protected:
    template <typename T>
    static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr &si, const std::string &new_name)
    {
        ompl::base::PlannerPtr planner(new T(si));
        if (!new_name.empty())
            planner->setName(new_name);
        return planner;
    }

    void setGoalState(const Eigen::VectorXd &qT, const double t, const double eps = 0);
    void preSolve();
    void postSolve();
    void getPath(Eigen::MatrixXd &traj, ompl::base::PlannerTerminationCondition &ptc);
    TimeIndexedRRTConnectInitializer init_;
    TimeIndexedSamplingProblem_ptr prob_;
    ompl::geometric::SimpleSetupPtr ompl_simple_setup_;
    ompl::base::StateSpacePtr state_space_;
    ConfiguredPlannerAllocator planner_allocator_;
    std::string algorithm_;
};
}
#endif /* TIME_INDEXED_RRT_CONNECT_TIMEINDEXEDRRTCONNECT_H_ */
