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
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/config/SelfConfig.h>
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

using namespace ompl;
class OMPLTimeIndexedRRTConnect : public base::Planner
{
public:
    /** \brief Constructor */
    OMPLTimeIndexedRRTConnect(const base::SpaceInformationPtr &si);

    virtual ~OMPLTimeIndexedRRTConnect();

    virtual void getPlannerData(base::PlannerData &data) const;

    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

    virtual void clear();

    /** \brief Set the range the planner is supposed to use.

	 This parameter greatly influences the runtime of the
	 algorithm. It represents the maximum length of a
	 motion to be added in the tree of motions. */
    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    /** \brief Get the range the planner is using */
    double getRange() const
    {
        return maxDistance_;
    }

    /** \brief Set a different nearest neighbors datastructure */
    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        tStart_.reset(new NN<Motion *>());
        tGoal_.reset(new NN<Motion *>());
    }

    virtual void setup();

protected:
    /** \brief Representation of a motion */
    class Motion
    {
    public:
        Motion() : root(NULL), state(NULL), parent(NULL)
        {
            parent = NULL;
            state = NULL;
        }

        Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL)
        {
        }

        ~Motion()
        {
        }

        const base::State *root;
        base::State *state;
        Motion *parent;
    };

    /** \brief A nearest-neighbor datastructure representing a tree of motions */
    typedef boost::shared_ptr<NearestNeighbors<Motion *> > TreeData;

    /** \brief Information attached to growing a tree of motions (used internally) */
    struct TreeGrowingInfo
    {
        base::State *xstate;
        Motion *xmotion;
        bool start;
        bool correct_time;
    };

    /** \brief The state of the tree after an attempt to extend it */
    enum GrowState
    {
        /// no progress has been made
        TRAPPED,
        /// progress has been made towards the randomly sampled state
        ADVANCED,
        /// the randomly sampled state was reached
        REACHED
    };

    /** \brief Free the memory allocated by this planner */
    void freeMemory();

    /** \brief Compute distance between motions (actually distance between contained states) */
    double distanceFunction(const Motion *a, const Motion *b) const
    {
        return si_->distance(a->state, b->state);
    }

    double forwardTimeDistance(const Motion *a, const Motion *b) const
    {
        const OMPLTimeIndexedRNStateSpace::StateType *sa = static_cast<const OMPLTimeIndexedRNStateSpace::StateType *>(a->state);
        const OMPLTimeIndexedRNStateSpace::StateType *sb = static_cast<const OMPLTimeIndexedRNStateSpace::StateType *>(b->state);

        double ta = sa->getTime().position, tb = sb->getTime().position;
        if ((reverse_check_ && tb < ta) || (!reverse_check_ && tb > ta))
            return 1e10;
        return si_->distance(a->state, b->state);
    }

    double reverseTimeDistance(const Motion *a, const Motion *b) const
    {
        const OMPLTimeIndexedRNStateSpace::StateType *sa = static_cast<const OMPLTimeIndexedRNStateSpace::StateType *>(a->state);
        const OMPLTimeIndexedRNStateSpace::StateType *sb = static_cast<const OMPLTimeIndexedRNStateSpace::StateType *>(b->state);

        double ta = sa->getTime().position, tb = sb->getTime().position;
        if ((reverse_check_ && tb > ta) || (!reverse_check_ && tb < ta))
            return 1e10;
        return si_->distance(a->state, b->state);
    }

    bool correctTime(const Motion *a, Motion *b, bool reverse, bool &changed) const
    {
        bool reverse_ = reverse_check_ ? !reverse : reverse;
        Eigen::VectorXd max_vel = Eigen::VectorXd::Ones(7);
        double dist = si_->distance(a->state, b->state);
        const OMPLTimeIndexedRNStateSpace::StateType *sa = static_cast<const OMPLTimeIndexedRNStateSpace::StateType *>(a->state);
        OMPLTimeIndexedRNStateSpace::StateType *sb = static_cast<OMPLTimeIndexedRNStateSpace::StateType *>(b->state);
        double ta = sa->getTime().position, tb = sb->getTime().position;
        if ((reverse_ && tb > ta) || (!reverse_ && ta > tb))
            return false;
        Eigen::VectorXd qa, qb;
        si_->getStateSpace()->as<OMPLTimeIndexedRNStateSpace>()->OMPLToExoticaState(a->state, qa, ta);
        si_->getStateSpace()->as<OMPLTimeIndexedRNStateSpace>()->OMPLToExoticaState(b->state, qb, tb);
        Eigen::VectorXd diff = (qb - qa).cwiseAbs();
        double min_dt = (diff.array() / max_vel.array()).maxCoeff();
        if (fabs(tb - ta) < min_dt)
        {
            tb = ta + (reverse_ ? -min_dt : min_dt);
            changed = true;
        }
        else
            changed = false;
        if (tb <= 0 || tb > si_->getStateSpace()->as<OMPLTimeIndexedRNStateSpace>()->prob_->T_)
            return false;
        si_->getStateSpace()->as<OMPLTimeIndexedRNStateSpace>()->ExoticaToOMPLState(qb, tb, b->state);
        return true;
    }

    /** \brief Grow a tree towards a random state */
    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

    /** \brief State sampler */
    base::StateSamplerPtr sampler_;

    /** \brief The start tree */
    TreeData tStart_;

    /** \brief The goal tree */
    TreeData tGoal_;

    /** \brief The maximum length of a motion to be added to a tree */
    double maxDistance_;

    /** \brief The random number generator */
    RNG rng_;

    /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
    std::pair<base::State *, base::State *> connectionPoint_;

    bool reverse_check_;
};
}
#endif /* TIME_INDEXED_RRT_CONNECT_TIMEINDEXEDRRTCONNECT_H_ */
