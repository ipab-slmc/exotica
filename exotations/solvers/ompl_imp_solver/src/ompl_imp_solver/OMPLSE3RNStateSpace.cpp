/*
 *  Created on: 22 Jun 2015
 *      Author: Yiming Yang
 * 
 * Copyright (c) 2016, University Of Edinburgh 
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

#include <ompl_imp_solver/OMPLSE3RNStateSpace.h>
namespace exotica
{
OMPLSE3RNStateSpace::OMPLSE3RNStateSpace(unsigned int dim, SamplingProblem_ptr &prob, OMPLsolverInitializer init)
    : OMPLBaseStateSpace(dim, prob, init), realvectordim_(dim), SO3Bounds_(3)
{
    setName("OMPLSE3RNCompoundStateSpace");
    addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 10.0);
    addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(dim)), 1.0);
    useGoal_ = init.UseGoalBias;
    rn_bias_percentage_ = init.GoalBias;
    weights_ = init.ImportanceWeights;
    unsigned int n = dim + 6;
    if (prob->getBounds().size() == 2 * n)
    {
        ompl::base::RealVectorBounds RNbounds(dim);
        ompl::base::RealVectorBounds SE3bounds(3);

        for (int i = 0; i < 3; i++)
        {
            SE3bounds.setHigh(i, prob->getBounds()[i + n]);
            SE3bounds.setLow(i, prob->getBounds()[i]);
        }
        SO3Bounds_.resize(3);
        for (int i = 3; i < 6; i++)
            SO3Bounds_.low[i - 3] = prob->getBounds()[i];
        for (int i = 3; i < 6; i++)
            SO3Bounds_.high[i - 3] = prob->getBounds()[i + n];
        setSE3StateSpaceBounds(SE3bounds);
        for (int i = 6; i < n; i++)
        {
            RNbounds.setHigh(i - 6, prob->getBounds()[i + n]);
            RNbounds.setLow(i - 6, prob->getBounds()[i]);
        }
        setRealVectorStateSpaceBounds(RNbounds);
    }
    else
    {
        ERROR(
            "State space bounds were not specified!\n"
            << prob->getBounds().size() << " " << n);
    }
    lock();
}

ob::StateSamplerPtr OMPLSE3RNStateSpace::allocDefaultStateSampler() const
{
    OMPLSE3RNStateSampler *state_sampler = new OMPLSE3RNStateSampler(this);
    return ob::StateSamplerPtr(state_sampler);
}

void OMPLSE3RNStateSpace::stateDebug(const Eigen::VectorXd &q) const
{
    //  TODO
}

unsigned int OMPLSE3RNStateSpace::getDimension() const
{
    return realvectordim_ + 6;
}

void OMPLSE3RNStateSpace::setRealVectorStateSpaceBounds(
    const ob::RealVectorBounds &bounds)
{
    bounds.check();
    getSubspace(1)->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

const ob::RealVectorBounds &OMPLSE3RNStateSpace::getRealVectorStateSpaceBounds() const
{
    return getSubspace(1)->as<ob::RealVectorStateSpace>()->getBounds();
}

void OMPLSE3RNStateSpace::setSE3StateSpaceBounds(
    const ob::RealVectorBounds &xyz, const double dist)
{
    xyz.check();
    getSubspace(0)->as<ob::SE3StateSpace>()->setBounds(xyz);
}

const ob::RealVectorBounds &OMPLSE3RNStateSpace::getSE3StateSpaceBounds() const
{
    return getSubspace(0)->as<ob::SE3StateSpace>()->getBounds();
}

void OMPLSE3RNStateSpace::setStart(const Eigen::VectorXd &start)
{
    start_ = start;
    base_dist_ = (goal_.segment(0, 2) - start_.segment(0, 2)).norm();
}

void OMPLSE3RNStateSpace::setGoal(const Eigen::VectorXd &goal)
{
    if (goal.rows() == realvectordim_ + 6)
    {
        goal_ = goal;
    }
}
void OMPLSE3RNStateSampler::sampleUniform(ob::State *state)
{
    OMPLSE3RNStateSpace::StateType *rstate =
        static_cast<OMPLSE3RNStateSpace::StateType *>(state);

    //	Sample for the SE3 space
    const OMPLSE3RNStateSpace *space =
        static_cast<const OMPLSE3RNStateSpace *>(space_);
    const ob::RealVectorBounds &se3_bounds =
        static_cast<const OMPLSE3RNStateSpace *>(space_)->getSE3StateSpaceBounds();
    ob::RealVectorBounds so3_bounds =
        static_cast<const OMPLSE3RNStateSpace *>(space_)->SO3Bounds_;
    rstate->SE3StateSpace().setXYZ(
        rng_.uniformReal(se3_bounds.low[0], se3_bounds.high[0]),
        rng_.uniformReal(se3_bounds.low[1], se3_bounds.high[1]),
        rng_.uniformReal(se3_bounds.low[2], se3_bounds.high[2]));
    //	Only rotates along Z, for base movement
    rstate->SE3StateSpace().rotation().setAxisAngle(0, 0, 1,
                                                    rng_.uniformReal(so3_bounds.low[0], so3_bounds.high[0]));
    rstate->SE3StateSpace().rotation().setAxisAngle(0, 1, 0,
                                                    rng_.uniformReal(so3_bounds.low[1], so3_bounds.high[1]));
    rstate->SE3StateSpace().rotation().setAxisAngle(1, 0, 0,
                                                    rng_.uniformReal(so3_bounds.low[2], so3_bounds.high[2]));
    //	Sample for the RN space
    const ob::RealVectorBounds &realvector_bounds =
        static_cast<const OMPLSE3RNStateSpace *>(space_)->getRealVectorStateSpaceBounds();
    const unsigned int dim = realvector_bounds.high.size();
    if (space->useGoal_)
    {
        Eigen::VectorXd tmp(2);
        tmp << rstate->SE3StateSpace().getX(), rstate->SE3StateSpace().getY();
        double percentage = (space->goal_.segment(0, 2) - tmp).norm() / space->base_dist_;
        if (percentage < space->rn_bias_percentage_)
        {
            for (unsigned int i = 0; i < dim; ++i)
                rstate->RealVectorStateSpace().values[i] = rng_.uniformReal(
                    realvector_bounds.low[i], realvector_bounds.high[i]);
        }
        else
            for (unsigned int i = 0; i < dim; ++i)
                rstate->RealVectorStateSpace().values[i] = space->start_(i + 6);
    }
    else
        for (unsigned int i = 0; i < dim; ++i)
            rstate->RealVectorStateSpace().values[i] = rng_.uniformReal(
                realvector_bounds.low[i], realvector_bounds.high[i]);
}

void OMPLSE3RNStateSampler::sampleUniformNear(ob::State *state,
                                              const ob::State *near, const double distance)
{
    //	First sample for the upper body
    OMPLSE3RNStateSpace::StateType *rstate =
        static_cast<OMPLSE3RNStateSpace::StateType *>(state);
    const OMPLSE3RNStateSpace::StateType *nstate =
        static_cast<const OMPLSE3RNStateSpace::StateType *>(near);
    const ob::RealVectorBounds &realvector_bounds =
        static_cast<const OMPLSE3RNStateSpace *>(space_)->getRealVectorStateSpaceBounds();
    const unsigned int dim = realvector_bounds.high.size();
    for (unsigned int i = 0; i < dim; ++i)
        rstate->RealVectorStateSpace().values[i] = rng_.uniformReal(
            std::max(realvector_bounds.low[i],
                     nstate->RealVectorStateSpace().values[i] - distance * weightImportance_[i + 6]),
            std::min(realvector_bounds.high[i],
                     nstate->RealVectorStateSpace().values[i] + distance * weightImportance_[i + 6]));
    //	Now sample for the SE3 space
    const ob::RealVectorBounds &se3_bounds =
        static_cast<const OMPLSE3RNStateSpace *>(space_)->getSE3StateSpaceBounds();
    rstate->SE3StateSpace().setX(
        rng_.uniformReal(
            std::max(se3_bounds.low[0],
                     nstate->SE3StateSpace().getX() - distance * weightImportance_[0]),
            std::min(se3_bounds.high[0],
                     nstate->SE3StateSpace().getX() + distance * weightImportance_[0])));
    rstate->SE3StateSpace().setY(
        rng_.uniformReal(
            std::max(se3_bounds.low[1],
                     nstate->SE3StateSpace().getY() - distance * weightImportance_[1]),
            std::min(se3_bounds.high[1],
                     nstate->SE3StateSpace().getY() + distance * weightImportance_[1])));
    rstate->SE3StateSpace().setZ(
        rng_.uniformReal(
            std::max(se3_bounds.low[2],
                     nstate->SE3StateSpace().getZ() - distance * weightImportance_[2]),
            std::min(se3_bounds.high[2],
                     nstate->SE3StateSpace().getZ() + distance * weightImportance_[2])));
    rstate->SE3StateSpace().rotation().setAxisAngle(0, 0, 1,
                                                    rng_.uniformReal(-1.57, 1.57));
}

void OMPLSE3RNStateSampler::sampleGaussian(ob::State *state,
                                           const ob::State *mean, const double stdDev)
{
    WARNING_NAMED("OMPLFullBodyStateSampler", "sampleGaussian not implemented");
}

void OMPLSE3RNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q,
                                             ompl::base::State *state) const
{
    OMPLSE3RNStateSpace::StateType *statetype =
        static_cast<OMPLSE3RNStateSpace::StateType *>(state);
    statetype->SE3StateSpace().setXYZ(q(0), q(1), q(2));
    KDL::Rotation tmp = KDL::Rotation::EulerZYX(q(3), q(4), q(5));
    tmp.GetQuaternion(statetype->SE3StateSpace().rotation().x,
                      statetype->SE3StateSpace().rotation().y,
                      statetype->SE3StateSpace().rotation().z,
                      statetype->SE3StateSpace().rotation().w);

    memcpy(statetype->RealVectorStateSpace().values,
           q.segment(6, q.rows() - 6).data(), sizeof(double) * (q.rows() - 6));
}

void OMPLSE3RNStateSpace::OMPLToExoticaState(
    const ompl::base::State *state, Eigen::VectorXd &q) const
{
    q.setZero(getDimension());
    const OMPLSE3RNStateSpace::StateType *statetype =
        static_cast<const OMPLSE3RNStateSpace::StateType *>(state);
    memcpy(q.segment(6, q.rows() - 6).data(),
           statetype->RealVectorStateSpace().values,
           sizeof(double) * (q.rows() - 6));
    q(0) = statetype->SE3StateSpace().getX();
    q(1) = statetype->SE3StateSpace().getY();
    q(2) = statetype->SE3StateSpace().getZ();

    KDL::Rotation tmp = KDL::Rotation::Quaternion(
        statetype->SE3StateSpace().rotation().x,
        statetype->SE3StateSpace().rotation().y,
        statetype->SE3StateSpace().rotation().z,
        statetype->SE3StateSpace().rotation().w);
    tmp.GetEulerZYX(q(3), q(4), q(5));
}
}
