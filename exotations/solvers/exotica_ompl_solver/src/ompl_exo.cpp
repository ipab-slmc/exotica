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

#include <exotica_ompl_solver/ompl_exo.h>

namespace exotica
{
OMPLStateValidityChecker::OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr &si, const SamplingProblemPtr &prob) : ompl::base::StateValidityChecker(si), prob_(prob)
{
}

bool OMPLStateValidityChecker::isValid(const ompl::base::State *state) const
{
    double tmp;
    return isValid(state, tmp);
}

bool OMPLStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
{
    Eigen::VectorXd q(prob_->N);

#if ROS_VERSION_MINIMUM(1, 12, 0)  // if ROS version >= ROS_KINETIC
    std::static_pointer_cast<OMPLStateSpace>(si_->getStateSpace())->OMPLToExoticaState(state, q);
#else
    boost::static_pointer_cast<OMPLStateSpace>(si_->getStateSpace())->OMPLToExoticaState(state, q);
#endif

    if (!prob_->IsValid(q))
    {
        dist = -1;
        return false;
    }
    return true;
}

OMPLRNStateSpace::OMPLRNStateSpace(OMPLSolverInitializer init) : OMPLStateSpace(init)
{
    setName("OMPLRNStateSpace");
}

ompl::base::StateSamplerPtr OMPLRNStateSpace::allocDefaultStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}

void OMPLRNStateSpace::SetBounds(SamplingProblemPtr &prob)
{
    unsigned int dim = prob->N;
    addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim)), 1.0);
    ompl::base::RealVectorBounds ompl_bounds(dim);
    auto bounds = prob->GetBounds();
    for (unsigned int i = 0; i < dim; ++i)
    {
        ompl_bounds.setHigh(i, bounds[i + dim]);
        ompl_bounds.setLow(i, bounds[i]);
    }
    getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->setBounds(ompl_bounds);
    setLongestValidSegmentFraction(init_.LongestValidSegmentFraction);
    lock();
}

void OMPLRNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const
{
    if (!state)
    {
        ThrowPretty("Invalid state!");
    }
    if (q.rows() != (int)getDimension())
    {
        ThrowPretty("State vector (" << q.rows() << ") and internal state (" << (int)getDimension() << ") dimension disagree");
    }
    memcpy(state->as<OMPLRNStateSpace::StateType>()->getRNSpace().values, q.data(), sizeof(double) * q.rows());
}

void OMPLRNStateSpace::OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const
{
    if (!state)
    {
        ThrowPretty("Invalid state!");
    }
    if (q.rows() != (int)getDimension())
        q.resize((int)getDimension());
    memcpy(q.data(), state->as<OMPLRNStateSpace::StateType>()->getRNSpace().values, sizeof(double) * q.rows());
}

void OMPLRNStateSpace::StateDebug(const Eigen::VectorXd &q) const
{
    //  TODO
}

OMPLSE3RNStateSpace::OMPLSE3RNStateSpace(OMPLSolverInitializer init) : OMPLStateSpace(init)
{
    setName("OMPLSE3RNStateSpace");
}

ompl::base::StateSamplerPtr OMPLSE3RNStateSpace::allocDefaultStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}

void OMPLSE3RNStateSpace::SetBounds(SamplingProblemPtr &prob)
{
    dim_ = prob->N;
    addSubspace(ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace()), 1.0);

    if (dim_ > 6)
    {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim_ - 6)), 1.0);
    }

    auto bounds = prob->GetBounds();
    if (bounds.size() == 2 * dim_)
    {
        // NB: We can only bound the xyz, rotation is unbounded (!)
        ompl::base::RealVectorBounds SE3bounds(3);
        for (int i = 0; i < 3; ++i)
        {
            SE3bounds.setHigh(i, bounds[i + dim_]);
            SE3bounds.setLow(i, bounds[i]);
        }
        getSubspace(0)->as<ompl::base::SE3StateSpace>()->setBounds(SE3bounds);
        WARNING_NAMED("OMPLSE3RNStateSpace::SetBounds", "Orientation bounds on SE(3) component ignored.");

        if (dim_ > 6)
        {
            ompl::base::RealVectorBounds RNbounds(dim_ - 6);
            for (unsigned int i = 6; i < dim_; ++i)
            {
                RNbounds.setHigh(i - 6, bounds[i + dim_]);
                RNbounds.setLow(i - 6, bounds[i]);
            }
            getSubspace(1)->as<ompl::base::RealVectorStateSpace>()->setBounds(RNbounds);
        }
    }
    else
    {
        ERROR("State space bounds were not specified!\n"
              << bounds.size() << " " << dim_);
    }
    setLongestValidSegmentFraction(init_.LongestValidSegmentFraction);
    lock();
}

void OMPLSE3RNStateSpace::StateDebug(const Eigen::VectorXd &q) const
{
}

void OMPLSE3RNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const
{
    OMPLSE3RNStateSpace::StateType *statetype = static_cast<OMPLSE3RNStateSpace::StateType *>(state);
    statetype->SE3StateSpace().setXYZ(q(0), q(1), q(2));
    KDL::Rotation tmp = KDL::Rotation::RPY(q(3), q(4), q(5));
    tmp.GetQuaternion(statetype->SE3StateSpace().rotation().x, statetype->SE3StateSpace().rotation().y, statetype->SE3StateSpace().rotation().z, statetype->SE3StateSpace().rotation().w);

    if (dim_ > 6)
    {
        memcpy(statetype->RealVectorStateSpace().values, q.segment(6, q.rows() - 6).data(), sizeof(double) * (q.rows() - 6));
    }
}

void OMPLSE3RNStateSpace::OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const
{
    q.setZero(getDimension());
    const OMPLSE3RNStateSpace::StateType *statetype = static_cast<const OMPLSE3RNStateSpace::StateType *>(state);
    q(0) = statetype->SE3StateSpace().getX();
    q(1) = statetype->SE3StateSpace().getY();
    q(2) = statetype->SE3StateSpace().getZ();

    KDL::Rotation tmp = KDL::Rotation::Quaternion(statetype->SE3StateSpace().rotation().x, statetype->SE3StateSpace().rotation().y, statetype->SE3StateSpace().rotation().z, statetype->SE3StateSpace().rotation().w);
    tmp.GetRPY(q(3), q(4), q(5));

    if (dim_ > 6)
    {
        memcpy(q.segment(6, q.rows() - 6).data(), statetype->RealVectorStateSpace().values, sizeof(double) * (q.rows() - 6));
    }
}

OMPLSE2RNStateSpace::OMPLSE2RNStateSpace(OMPLSolverInitializer init) : OMPLStateSpace(init)
{
    setName("OMPLSE2RNStateSpace");
}

ompl::base::StateSamplerPtr OMPLSE2RNStateSpace::allocDefaultStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}

void OMPLSE2RNStateSpace::SetBounds(SamplingProblemPtr &prob)
{
    dim_ = prob->N;
    addSubspace(ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace()), 1.0);
    if (dim_ > 3)
    {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim_ - 3)), 1.0);
    }

    auto bounds = prob->GetBounds();
    if (bounds.size() == 2 * dim_)
    {
        ompl::base::RealVectorBounds SE2bounds(2);
        for (int i = 0; i < 3; ++i)
        {
            SE2bounds.setHigh(i, bounds[i + dim_]);
            SE2bounds.setLow(i, bounds[i]);
        }
        getSubspace(0)->as<ompl::base::SE2StateSpace>()->setBounds(SE2bounds);
        WARNING_NAMED("OMPLSE2RNStateSpace::SetBounds", "Yaw bounds on SE(2) component ignored.");

        if (dim_ > 3)
        {
            ompl::base::RealVectorBounds RNbounds(dim_ - 3);
            for (unsigned int i = 3; i < dim_; ++i)
            {
                RNbounds.setHigh(i - 3, prob->GetBounds()[i + dim_]);
                RNbounds.setLow(i - 3, prob->GetBounds()[i]);
            }
            getSubspace(1)->as<ompl::base::RealVectorStateSpace>()->setBounds(RNbounds);
        }
    }
    else
    {
        ERROR("State space bounds were not specified!" << std::endl
                                                       << bounds.size() << " " << dim_);
    }
    setLongestValidSegmentFraction(init_.LongestValidSegmentFraction);
    lock();
}

void OMPLSE2RNStateSpace::StateDebug(const Eigen::VectorXd &q) const
{
}

void OMPLSE2RNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const
{
    OMPLSE2RNStateSpace::StateType *statetype = static_cast<OMPLSE2RNStateSpace::StateType *>(state);
    statetype->SE2StateSpace().setXY(q(0), q(1));
    statetype->SE2StateSpace().setYaw(q(2));

    if (dim_ > 3)
    {
        memcpy(statetype->RealVectorStateSpace().values, q.segment(3, q.rows() - 3).data(), sizeof(double) * (q.rows() - 3));
    }
}

void OMPLSE2RNStateSpace::OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const
{
    q.setZero(getDimension());
    const OMPLSE2RNStateSpace::StateType *statetype = static_cast<const OMPLSE2RNStateSpace::StateType *>(state);
    q(0) = statetype->SE2StateSpace().getX();
    q(1) = statetype->SE2StateSpace().getY();
    q(2) = statetype->SE2StateSpace().getYaw();

    if (dim_ > 3)
    {
        memcpy(q.segment(3, q.rows() - 3).data(), statetype->RealVectorStateSpace().values, sizeof(double) * (q.rows() - 3));
    }
}

OMPLDubinsRNStateSpace::OMPLDubinsRNStateSpace(OMPLSolverInitializer init) : OMPLStateSpace(init)
{
    setName("OMPLDubinsRNStateSpace");
}

ompl::base::StateSamplerPtr OMPLDubinsRNStateSpace::allocDefaultStateSampler() const
{
    return CompoundStateSpace::allocDefaultStateSampler();
}

void OMPLDubinsRNStateSpace::SetBounds(SamplingProblemPtr &prob)
{
    dim_ = prob->N;
    addSubspace(ompl::base::StateSpacePtr(new ompl::base::DubinsStateSpace(init_.DubinsStateSpaceTurningRadius, init_.DubinsStateSpaceIsSymmetric)), 1.0);
    if (dim_ > 3)
    {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim_ - 3)), 1.0);
    }

    auto bounds = prob->GetBounds();
    if (bounds.size() == 2 * dim_)
    {
        ompl::base::RealVectorBounds Dubinsbounds(2);
        for (int i = 0; i < 3; ++i)
        {
            Dubinsbounds.setHigh(i, bounds[i + dim_]);
            Dubinsbounds.setLow(i, bounds[i]);
        }
        getSubspace(0)->as<ompl::base::DubinsStateSpace>()->setBounds(Dubinsbounds);
        WARNING_NAMED("OMPLDubinsRNStateSpace::SetBounds", "Yaw bounds on SE(2) component ignored.");

        if (dim_ > 3)
        {
            ompl::base::RealVectorBounds RNbounds(dim_ - 3);
            for (unsigned int i = 3; i < dim_; ++i)
            {
                RNbounds.setHigh(i - 3, prob->GetBounds()[i + dim_]);
                RNbounds.setLow(i - 3, prob->GetBounds()[i]);
            }
            getSubspace(1)->as<ompl::base::RealVectorStateSpace>()->setBounds(RNbounds);
        }
    }
    else
    {
        ERROR("State space bounds were not specified!" << std::endl
                                                       << bounds.size() << " " << dim_);
    }
    setLongestValidSegmentFraction(init_.LongestValidSegmentFraction);
    lock();
}

void OMPLDubinsRNStateSpace::StateDebug(const Eigen::VectorXd &q) const
{
}

void OMPLDubinsRNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const
{
    OMPLDubinsRNStateSpace::StateType *statetype = static_cast<OMPLDubinsRNStateSpace::StateType *>(state);
    statetype->DubinsStateSpace().setXY(q(0), q(1));
    statetype->DubinsStateSpace().setYaw(q(2));

    if (dim_ > 3)
    {
        memcpy(statetype->RealVectorStateSpace().values, q.segment(3, q.rows() - 3).data(), sizeof(double) * (q.rows() - 3));
    }
}

void OMPLDubinsRNStateSpace::OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const
{
    q.setZero(getDimension());
    const OMPLDubinsRNStateSpace::StateType *statetype = static_cast<const OMPLDubinsRNStateSpace::StateType *>(state);
    q(0) = statetype->DubinsStateSpace().getX();
    q(1) = statetype->DubinsStateSpace().getY();
    q(2) = statetype->DubinsStateSpace().getYaw();

    if (dim_ > 3)
    {
        memcpy(q.segment(3, q.rows() - 3).data(), statetype->RealVectorStateSpace().values, sizeof(double) * (q.rows() - 3));
    }
}
}  // namespace exotica
