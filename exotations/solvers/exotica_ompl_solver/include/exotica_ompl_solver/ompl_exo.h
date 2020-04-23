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

#ifndef EXOTICA_OMPL_SOLVER_OMPL_EXO_H_
#define EXOTICA_OMPL_SOLVER_OMPL_EXO_H_

#include <exotica_core/problems/sampling_problem.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <exotica_ompl_solver/ompl_solver_initializer.h>

#if OMPL_VERSION_VALUE >= 1004000  // Version greater than 1.4.0
typedef Eigen::Ref<Eigen::VectorXd> OMPLProjection;
#else  // All other versions
typedef ompl::base::EuclideanProjection &OMPLProjection;
#endif

namespace exotica
{
class OMPLStateSpace : public ompl::base::CompoundStateSpace
{
public:
    OMPLStateSpace(OMPLSolverInitializer init) : ompl::base::CompoundStateSpace(), init_(init)
    {
    }

    virtual void SetBounds(SamplingProblemPtr &prob) = 0;
    virtual void ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const = 0;
    virtual void OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const = 0;

    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const = 0;
    virtual void StateDebug(const Eigen::VectorXd &q) const = 0;

protected:
    OMPLSolverInitializer init_;
};

class OMPLStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr &si, const SamplingProblemPtr &prob);

    bool isValid(const ompl::base::State *state) const override;

    bool isValid(const ompl::base::State *state, double &dist) const override;

protected:
    SamplingProblemPtr prob_;
};

class OMPLRNStateSpace : public OMPLStateSpace
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
    };
    OMPLRNStateSpace(OMPLSolverInitializer init);

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;
    void SetBounds(SamplingProblemPtr &prob) override;
    void ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const override;
    void OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const override;
    void StateDebug(const Eigen::VectorXd &q) const override;
};

class OMPLSE3RNStateSpace : public OMPLStateSpace
{
public:
    class StateType : public ompl::base::CompoundStateSpace::StateType
    {
    public:
        StateType() : CompoundStateSpace::StateType()
        {
        }

        const ompl::base::RealVectorStateSpace::StateType &RealVectorStateSpace() const
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(1);
        }

        ompl::base::RealVectorStateSpace::StateType &RealVectorStateSpace()
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(1);
        }

        const ompl::base::SE3StateSpace::StateType &SE3StateSpace() const
        {
            return *as<ompl::base::SE3StateSpace::StateType>(0);
        }
        ompl::base::SE3StateSpace::StateType &SE3StateSpace()
        {
            return *as<ompl::base::SE3StateSpace::StateType>(0);
        }
    };
    OMPLSE3RNStateSpace(OMPLSolverInitializer init);

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;
    void SetBounds(SamplingProblemPtr &prob) override;
    void ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const override;
    void OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const override;
    void StateDebug(const Eigen::VectorXd &q) const override;

private:
    unsigned int dim_ = 6;
};

class OMPLSE2RNStateSpace : public OMPLStateSpace
{
public:
    class StateType : public ompl::base::CompoundStateSpace::StateType
    {
    public:
        StateType() : CompoundStateSpace::StateType()
        {
        }

        const ompl::base::RealVectorStateSpace::StateType &RealVectorStateSpace() const
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(1);
        }

        ompl::base::RealVectorStateSpace::StateType &RealVectorStateSpace()
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(1);
        }

        const ompl::base::SE2StateSpace::StateType &SE2StateSpace() const
        {
            return *as<ompl::base::SE2StateSpace::StateType>(0);
        }
        ompl::base::SE2StateSpace::StateType &SE2StateSpace()
        {
            return *as<ompl::base::SE2StateSpace::StateType>(0);
        }
    };
    OMPLSE2RNStateSpace(OMPLSolverInitializer init);

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;
    void SetBounds(SamplingProblemPtr &prob) override;
    void ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const override;
    void OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const override;
    void StateDebug(const Eigen::VectorXd &q) const override;

private:
    unsigned int dim_ = 3;
};

class OMPLDubinsRNStateSpace : public OMPLStateSpace
{
public:
    class StateType : public ompl::base::CompoundStateSpace::StateType
    {
    public:
        StateType() : CompoundStateSpace::StateType()
        {
        }

        const ompl::base::RealVectorStateSpace::StateType &RealVectorStateSpace() const
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(1);
        }

        ompl::base::RealVectorStateSpace::StateType &RealVectorStateSpace()
        {
            return *as<ompl::base::RealVectorStateSpace::StateType>(1);
        }

        const ompl::base::DubinsStateSpace::StateType &DubinsStateSpace() const
        {
            return *as<ompl::base::DubinsStateSpace::StateType>(0);
        }
        ompl::base::SE2StateSpace::StateType &DubinsStateSpace()
        {
            return *as<ompl::base::DubinsStateSpace::StateType>(0);
        }
    };
    OMPLDubinsRNStateSpace(OMPLSolverInitializer init);

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;
    void SetBounds(SamplingProblemPtr &prob) override;
    void ExoticaToOMPLState(const Eigen::VectorXd &q, ompl::base::State *state) const override;
    void OMPLToExoticaState(const ompl::base::State *state, Eigen::VectorXd &q) const override;
    void StateDebug(const Eigen::VectorXd &q) const override;

private:
    unsigned int dim_ = 3;
};

class OMPLRNProjection : public ompl::base::ProjectionEvaluator
{
public:
    OMPLRNProjection(const ompl::base::StateSpacePtr &space,
                     const std::vector<int> &vars)
        : ompl::base::ProjectionEvaluator(space), variables_(vars)
    {
    }

    ~OMPLRNProjection()
    {
    }

    unsigned int getDimension(void) const override
    {
        return variables_.size();
    }

    void defaultCellSizes() override
    {
        cellSizes_.clear();
        cellSizes_.resize(variables_.size(), 0.1);
    }

    void project(const ompl::base::State *state,
                 OMPLProjection projection) const override
    {
        for (std::size_t i = 0; i < variables_.size(); ++i)
            projection(i) =
                state->as<exotica::OMPLRNStateSpace::StateType>()->getRNSpace().values[variables_[i]];
    }

private:
    std::vector<int> variables_;
};

class OMPLSE3RNProjection : public ompl::base::ProjectionEvaluator
{
public:
    OMPLSE3RNProjection(const ompl::base::StateSpacePtr &space,
                        const std::vector<int> &vars)
        : ompl::base::ProjectionEvaluator(space), variables_(vars)
    {
    }

    ~OMPLSE3RNProjection()
    {
    }

    unsigned int getDimension(void) const override
    {
        return variables_.size();
    }

    void defaultCellSizes() override
    {
        cellSizes_.clear();
        cellSizes_.resize(variables_.size(), 0.1);
    }

    void project(const ompl::base::State *state,
                 OMPLProjection projection) const override
    {
        for (std::size_t i = 0; i < variables_.size(); ++i)
            projection(i) =
                state->as<exotica::OMPLSE3RNStateSpace::StateType>()->RealVectorStateSpace().values[variables_[i]];
    }

private:
    std::vector<int> variables_;
};

class OMPLSE2RNProjection : public ompl::base::ProjectionEvaluator
{
public:
    OMPLSE2RNProjection(const ompl::base::StateSpacePtr &space,
                        const std::vector<int> &vars)
        : ompl::base::ProjectionEvaluator(space), variables_(vars)
    {
    }

    ~OMPLSE2RNProjection()
    {
    }

    unsigned int getDimension(void) const override
    {
        return variables_.size();
    }

    void defaultCellSizes() override
    {
        cellSizes_.clear();
        cellSizes_.resize(variables_.size(), 0.1);
    }

    void project(const ompl::base::State *state,
                 OMPLProjection projection) const override
    {
        for (std::size_t i = 0; i < variables_.size(); ++i)
            projection(i) =
                state->as<exotica::OMPLSE2RNStateSpace::StateType>()->RealVectorStateSpace().values[variables_[i]];
    }

private:
    std::vector<int> variables_;
};
}  // namespace exotica

#endif  // EXOTICA_OMPL_SOLVER_OMPL_EXO_H_
