/*
 * ompl_exo.cpp
 *
 *  Created on: 10 Oct 2017
 *      Author: yiming
 */

#include <ompl_solver/ompl_exo.h>

namespace exotica {
OMPLBaseStateSpace::OMPLBaseStateSpace(unsigned int dim,
		SamplingProblem_ptr &prob, OMPLsolverInitializer init) :
		ompl::base::CompoundStateSpace(), prob_(prob) {

}

OMPLStateValidityChecker::OMPLStateValidityChecker(
		const ompl::base::SpaceInformationPtr &si, const SamplingProblem_ptr &prob) :
		ompl::base::StateValidityChecker(si), prob_(prob) {
}

bool OMPLStateValidityChecker::isValid(const ompl::base::State *state) const {
	double tmp;
	return isValid(state, tmp);
}

bool OMPLStateValidityChecker::isValid(const ompl::base::State *state,
		double &dist) const {
	Eigen::VectorXd q(prob_->N);

#ifdef ROS_KINETIC
	std::static_pointer_cast<OMPLBaseStateSpace>(si_->getStateSpace())->OMPLToExoticaState(state, q);
#else
	boost::static_pointer_cast<OMPLBaseStateSpace>(si_->getStateSpace())->OMPLToExoticaState(state, q);
#endif

	if (!prob_->isValid(q)) {
		dist = -1;
		return false;
	}
	return true;
}

OMPLRNStateSpace::OMPLRNStateSpace(unsigned int dim, SamplingProblem_ptr &prob,
		OMPLsolverInitializer init) :
		OMPLBaseStateSpace(dim, prob, init) {
	setName("OMPLRNStateSpace");
	addSubspace(
			ompl::base::StateSpacePtr(
					new ompl::base::RealVectorStateSpace(dim)), 1.0);
	ompl::base::RealVectorBounds bounds(dim);
	for (int i = 0; i < dim; i++) {
		bounds.setHigh(i, prob->getBounds()[i + dim]);
		bounds.setLow(i, prob->getBounds()[i]);
	}
	getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
	lock();
}

ompl::base::StateSamplerPtr OMPLRNStateSpace::allocDefaultStateSampler() const {
	return CompoundStateSpace::allocDefaultStateSampler();
}

void OMPLRNStateSpace::ExoticaToOMPLState(const Eigen::VectorXd &q,
		ompl::base::State *state) const {
	if (!state) {
		throw_pretty("Invalid state!");
	}
	if (q.rows() != (int) getDimension()) {
		throw_pretty(
				"State vector (" << q.rows() << ") and internal state (" << (int)getDimension() << ") dimension disagree");
	}
	memcpy(state->as<OMPLRNStateSpace::StateType>()->getRNSpace().values,
			q.data(), sizeof(double) * q.rows());
}

void OMPLRNStateSpace::OMPLToExoticaState(const ompl::base::State *state,
		Eigen::VectorXd &q) const {
	if (!state) {
		throw_pretty("Invalid state!");
	}
	if (q.rows() != (int) getDimension())
		q.resize((int) getDimension());
	memcpy(q.data(),
			state->as<OMPLRNStateSpace::StateType>()->getRNSpace().values,
			sizeof(double) * q.rows());
}

void OMPLRNStateSpace::stateDebug(const Eigen::VectorXd &q) const {
	//  TODO
}
}
