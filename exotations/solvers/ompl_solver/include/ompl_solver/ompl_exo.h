/*
 * ompl_exo.h
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#ifndef INCLUDE_OMPL_OMPL_EXO_H_
#define INCLUDE_OMPL_OMPL_EXO_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <exotica/Problems/SamplingProblem.h>
#include <ompl_solver/OMPLsolverInitializer.h>

namespace exotica {
class OMPLBaseStateSpace: public ompl::base::CompoundStateSpace {
public:
	OMPLBaseStateSpace(unsigned int dim, SamplingProblem_ptr &prob,
			OMPLsolverInitializer init);

	virtual void ExoticaToOMPLState(const Eigen::VectorXd &q,
			ompl::base::State *state) const = 0;
	virtual void OMPLToExoticaState(const ompl::base::State *state,
			Eigen::VectorXd &q) const = 0;

	virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const = 0;
	virtual void stateDebug(const Eigen::VectorXd &q) const = 0;

protected:
	SamplingProblem_ptr prob_;
};

class OMPLStateValidityChecker: public ompl::base::StateValidityChecker {
public:
	OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
			const SamplingProblem_ptr &prob);

	virtual bool isValid(const ompl::base::State *state) const;

	virtual bool isValid(const ompl::base::State *state, double &dist) const;

protected:
	SamplingProblem_ptr prob_;
};

class OMPLRNStateSpace: public OMPLBaseStateSpace {
public:
	class StateType: public ompl::base::CompoundStateSpace::StateType {
	public:
		StateType() :
				CompoundStateSpace::StateType() {
		}

		const ompl::base::RealVectorStateSpace::StateType &getRNSpace() const {
			return *as<ompl::base::RealVectorStateSpace::StateType>(0);
		}

		ompl::base::RealVectorStateSpace::StateType &getRNSpace() {
			return *as<ompl::base::RealVectorStateSpace::StateType>(0);
		}
	};
	OMPLRNStateSpace(unsigned int dim, SamplingProblem_ptr &prob,
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
