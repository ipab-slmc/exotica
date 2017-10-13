/*
 * ompl_exo.h
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#ifndef INCLUDE_OMPL_OMPL_EXO_H_
#define INCLUDE_OMPL_OMPL_EXO_H_

#include <exotica/Problems/SamplingProblem.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl_solver/OMPLsolverInitializer.h>

namespace exotica
{
class OMPLStateSpace : public ompl::base::CompoundStateSpace
{
public:
    OMPLStateSpace(SamplingProblem_ptr &prob) : ompl::base::CompoundStateSpace(), prob_(prob)
    {
    }

    virtual void ExoticaToOMPLState(const Eigen::VectorXd &q,
                                    ompl::base::State *state) const = 0;
    virtual void OMPLToExoticaState(const ompl::base::State *state,
                                    Eigen::VectorXd &q) const = 0;

    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const = 0;
    virtual void stateDebug(const Eigen::VectorXd &q) const = 0;

protected:
    SamplingProblem_ptr prob_;
};

class OMPLStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                             const SamplingProblem_ptr &prob);

    virtual bool isValid(const ompl::base::State *state) const;

    virtual bool isValid(const ompl::base::State *state, double &dist) const;

protected:
    SamplingProblem_ptr prob_;
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
    OMPLRNStateSpace(SamplingProblem_ptr &prob,
                     OMPLsolverInitializer init);

    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
    virtual void ExoticaToOMPLState(const Eigen::VectorXd &q,
                                    ompl::base::State *state) const;
    virtual void OMPLToExoticaState(const ompl::base::State *state,
                                    Eigen::VectorXd &q) const;
    virtual void stateDebug(const Eigen::VectorXd &q) const;
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
    OMPLSE3RNStateSpace(SamplingProblem_ptr &prob,
                        OMPLsolverInitializer init);

    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
    virtual void ExoticaToOMPLState(const Eigen::VectorXd &q,
                                    ompl::base::State *state) const;
    virtual void OMPLToExoticaState(const ompl::base::State *state,
                                    Eigen::VectorXd &q) const;
    virtual void stateDebug(const Eigen::VectorXd &q) const;
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
    OMPLSE2RNStateSpace(SamplingProblem_ptr &prob,
                        OMPLsolverInitializer init);

    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
    virtual void ExoticaToOMPLState(const Eigen::VectorXd &q,
                                    ompl::base::State *state) const;
    virtual void OMPLToExoticaState(const ompl::base::State *state,
                                    Eigen::VectorXd &q) const;
    virtual void stateDebug(const Eigen::VectorXd &q) const;
};
} /* namespace exotica */

#endif /* INCLUDE_OMPL_OMPL_EXO_H_ */
