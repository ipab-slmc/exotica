/*
 * OMPLUpperBodyPelvisStateSpace.cpp
 *
 *  Created on: 22 Jun 2015
 *      Author: yiming
 */

#include <ompl_solver/OMPLSE3RNCompoundStateSpace.h>
#include "kinematic_maps/CoM.h"
namespace exotica
{

	OMPLSE3RNCompoundStateSpace::OMPLSE3RNCompoundStateSpace(unsigned int dim) :
					ob::CompoundStateSpace(),
					pelvis_angle_bound_(0.1),
					upperbody_dim_(dim),
					pelvis_xyx_bounds_(3)
	{
		setName("OMPLUpperBodyPelvisStateSpace");
		addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(dim)), 1.0);
		addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.0);
		lock();
	}

	OMPLSE3RNCompoundStateSpace::~OMPLSE3RNCompoundStateSpace()
	{

	}

	ob::StateSamplerPtr OMPLSE3RNCompoundStateSpace::allocDefaultStateSampler()
	{
		OMPLSE3RNCompoundStateSpace *ss = new OMPLSE3RNCompoundStateSpace(this);
		return ob::StateSamplerPtr(ss);
	}
	boost::shared_ptr<OMPLSE3RNCompoundStateSpace> OMPLSE3RNCompoundStateSpace::FromProblem(
			OMPLProblem_ptr prob)
	{
		unsigned int n = prob->getSpaceDim();
		boost::shared_ptr<OMPLSE3RNCompoundStateSpace> ret;

		if (n <= 0)
		{
			ERROR("State space size error!");
			return ret;
		}
		ret.reset(new OMPLSE3RNCompoundStateSpace(n));
		ompl::base::RealVectorBounds bounds(n);
		if (prob->getBounds().size() == 2 * n)
		{
			for (int i = 0; i < n; i++)
			{
				bounds.setHigh(i, prob->getBounds()[i + n]);
				bounds.setLow(i, prob->getBounds()[i]);
			}
		}
		else
		{
			WARNING("State space bounds were not specified!\n"<< prob->getBounds().size() << " " << n);
		}
		ret->setUpperBodyBounds(bounds);

		if (prob->full_body_plan_->data)
		{
			if (prob->getTaskMaps().find("CoMMap") == prob->getTaskMaps().end())
			{
				ERROR("A 'CoMMap'(TaskMap::CoM) is required for OMPLUpperBodyPelvisStateSpace");
				return ret;
			}
			else
			{
				HIGHLIGHT_NAMED("OMPLUpperBodyPelvisStateSpace", "OMPL State set to Fullbody mode");
				boost::shared_ptr<exotica::CoM> com_map =
						boost::static_pointer_cast<exotica::CoM>(prob->getTaskMaps().at("CoMMap"));
				unsigned int size = com_map->getBounds()->data.size() / 2;
				if (com_map->getBounds()->data.size() != 6)
				{
					WARNING_NAMED("OMPLUpperBodyPelvisStateSpace", "COM has bounds size "<<com_map->getBounds()->data.size()<<". Suppose to be 6");
					size = 2;
				}
				ompl::base::RealVectorBounds com_bounds(size);
				for (int i = 0; i < size; i++)
				{
					com_bounds.setLow(i, com_map->getBounds()->data[2 * i]);
					com_bounds.setHigh(i, com_map->getBounds()->data[2 * i + 1]);
				}
				HIGHLIGHT("Pelvis Bounds ["<<com_bounds.low[0]<<","<<com_bounds.high[0]<<"] ["<<com_bounds.low[1]<<","<<com_bounds.high[1]<<"] ["<<com_bounds.low[2]<<","<<com_bounds.high[2]<<"].")
				ret->setPelvisBounds(com_bounds, 0.1);
			}
		}
		else
		{
			HIGHLIGHT_NAMED("OMPLUpperBodyPelvisStateSpace", "OMPL State set to NON-Fullbody mode");
		}
		return ret;
	}
	unsigned int OMPLSE3RNCompoundStateSpace::getDimension() const
	{
		return upperbody_dim_;
	}
	void OMPLSE3RNCompoundStateSpace::setUpperBodyBounds(const ob::RealVectorBounds &bounds)
	{
		getSubspace(0)->as<ob::RealVectorStateSpace>()->setBounds(bounds);
	}

	const ob::RealVectorBounds & OMPLSE3RNCompoundStateSpace::getUpperBodyBounds() const
	{
		return getSubspace(0)->as<ob::RealVectorStateSpace>()->getBounds();
	}

	void OMPLSE3RNCompoundStateSpace::setPelvisBounds(const ob::RealVectorBounds &xyz,
			const double dist)
	{
		xyz.check();
		getSubspace(1)->as<ob::SE3StateSpace>()->setBounds(xyz);
		pelvis_xyx_bounds_ = xyz;
		pelvis_angle_bound_ = dist;
	}

	const ob::RealVectorBounds & OMPLSE3RNCompoundStateSpace::getPelvisPositionBounds() const
	{
		return pelvis_xyx_bounds_;
	}

	const double & OMPLSE3RNCompoundStateSpace::getPelvisRotationBound() const
	{
		return pelvis_angle_bound_;
	}

	void OMPLSE3RNCompoundStateSampler::sampleUniform(ob::State *state)
	{
		HIGHLIGHT("Calling sampleUniform");
		//	First sample for the upper body
		OMPLSE3RNCompoundStateSpace::StateType *rstate =
				static_cast<OMPLSE3RNCompoundStateSpace::StateType*>(state);
		const ob::RealVectorBounds &upperbody_bounds =
				static_cast<const OMPLSE3RNCompoundStateSpace*>(space_)->getUpperBodyBounds();
		const unsigned int dim = upperbody_bounds.high.size();
		for (unsigned int i = 0; i < dim; ++i)
			rstate->upperBodyConfiguration().values[i] =
					rng_.uniformReal(upperbody_bounds.low[i], upperbody_bounds.high[i]);

		//	Now sample for the pelvis
		const ob::RealVectorBounds &pelvis_bounds =
				static_cast<const OMPLSE3RNCompoundStateSpace*>(space_)->getPelvisPositionBounds();
		const double &pelvis_rot =
				static_cast<const OMPLSE3RNCompoundStateSpace*>(space_)->getPelvisRotationBound();
		rstate->pelvisPose().setXYZ(rng_.uniformReal(pelvis_bounds.low[0], pelvis_bounds.high[0]), rng_.uniformReal(pelvis_bounds.low[1], pelvis_bounds.high[1]), rng_.uniformReal(pelvis_bounds.low[2], pelvis_bounds.high[2]));
		rstate->pelvisPose().rotation().setAxisAngle(0, 0, 1, rng_.uniformReal(0, pelvis_rot));
	}

	void OMPLSE3RNCompoundStateSampler::sampleUniformNear(ob::State *state, const ob::State *near,
			const double distance)
	{
		WARNING_NAMED("OMPLFullBodyStateSampler", "sampleUniformNear not implemented");
	}

	void OMPLSE3RNCompoundStateSampler::sampleGaussian(ob::State *state, const ob::State * mean,
			const double stdDev)
	{
		WARNING_NAMED("OMPLFullBodyStateSampler", "sampleGaussian not implemented");
	}

	EReturn OMPLSE3RNCompoundStateSpace::OMPLStateToEigen(const ob::State *ompl,
			Eigen::VectorXd &eigen)
	{

		return SUCCESS;
	}

	EReturn OMPLSE3RNCompoundStateSpace::EigenToOMPLState(const Eigen::VectorXd &eigen,
			ob::State *ompl)
	{
		return SUCCESS;
	}
}

