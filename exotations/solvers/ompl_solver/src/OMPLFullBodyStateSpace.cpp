/*
 * OMPLFullBodyStateSpace.cpp
 *
 *  Created on: 22 Jun 2015
 *      Author: yiming
 */

#include "ompl_solver/OMPLFullBodyStateSpace.h"
#include "kinematic_maps/CoM.h"
namespace exotica
{

	OMPLFullBodyStateSpace::OMPLFullBodyStateSpace(unsigned int dim, bool fullbody) :
					ob::CompoundStateSpace(),
					pelvis_angle_bound_(0.1),
					fullbody_(fullbody),
					upperbody_dim_(dim),
					pelvis_xyx_bounds_(3)
	{
		setName("OMPLFullBodyStateSpace");
		addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(dim)), 1.0);
		if (fullbody)
			addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.0);
		lock();
	}

	OMPLFullBodyStateSpace::~OMPLFullBodyStateSpace()
	{

	}

	ob::StateSamplerPtr OMPLFullBodyStateSpace::allocDefaultStateSampler()
	{
		OMPLFullBodyStateSampler *ss = new OMPLFullBodyStateSampler(this);
		return ob::StateSamplerPtr(ss);
	}
	boost::shared_ptr<OMPLFullBodyStateSpace> OMPLFullBodyStateSpace::FromProblem(
			OMPLProblem_ptr prob)
	{
		unsigned int n = prob->getSpaceDim();
		boost::shared_ptr<OMPLFullBodyStateSpace> ret;

		if (n <= 0)
		{
			ERROR("State space size error!");
			return ret;
		}
		ret.reset(new OMPLFullBodyStateSpace(n, prob->full_body_plan_->data));
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
				ERROR("A 'CoMMap'(TaskMap::CoM) is required for OMPLFullBodyStateSpace");
				return ret;
			}
			else
			{
				HIGHLIGHT_NAMED("OMPLFullBodyStateSpace", "OMPL State set to Fullbody mode");
				boost::shared_ptr<exotica::CoM> com_map =
						boost::static_pointer_cast<exotica::CoM>(prob->getTaskMaps().at("CoMMap"));
				unsigned int size = com_map->getBounds()->data.size() / 2;
				if (com_map->getBounds()->data.size() != 6)
				{
					WARNING_NAMED("OMPLFullBodyStateSpace", "COM has bounds size "<<com_map->getBounds()->data.size()<<". Suppose to be 6");
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
			HIGHLIGHT_NAMED("OMPLFullBodyStateSpace", "OMPL State set to NON-Fullbody mode");
		}
		return ret;
	}
	unsigned int OMPLFullBodyStateSpace::getDimension() const
	{
		return upperbody_dim_;
	}
	void OMPLFullBodyStateSpace::setUpperBodyBounds(const ob::RealVectorBounds &bounds)
	{
		getSubspace(0)->as<ob::RealVectorStateSpace>()->setBounds(bounds);
	}

	const ob::RealVectorBounds & OMPLFullBodyStateSpace::getUpperBodyBounds() const
	{
		return getSubspace(0)->as<ob::RealVectorStateSpace>()->getBounds();
	}

	void OMPLFullBodyStateSpace::setPelvisBounds(const ob::RealVectorBounds &xyz, const double dist)
	{
		xyz.check();
		getSubspace(1)->as<ob::SE3StateSpace>()->setBounds(xyz);
		pelvis_xyx_bounds_ = xyz;
		pelvis_angle_bound_ = dist;
	}

	const ob::RealVectorBounds & OMPLFullBodyStateSpace::getPelvisPositionBounds() const
	{
		return pelvis_xyx_bounds_;
	}

	const double & OMPLFullBodyStateSpace::getPelvisRotationBound() const
	{
		return pelvis_angle_bound_;
	}

	void OMPLFullBodyStateSampler::sampleUniform(ob::State *state)
	{
		HIGHLIGHT("Calling sampleUniform");
		//	First sample for the upper body
		OMPLFullBodyStateSpace::StateType *rstate =
				static_cast<OMPLFullBodyStateSpace::StateType*>(state);
		const ob::RealVectorBounds &upperbody_bounds =
				static_cast<const OMPLFullBodyStateSpace*>(space_)->getUpperBodyBounds();
		const unsigned int dim = upperbody_bounds.high.size();
		for (unsigned int i = 0; i < dim; ++i)
			rstate->upperBodyConfiguration().values[i] =
					rng_.uniformReal(upperbody_bounds.low[i], upperbody_bounds.high[i]);

		//	Now sample for the pelvis
		if (static_cast<const OMPLFullBodyStateSpace*>(space_)->fullbody_)
		{
			const ob::RealVectorBounds &pelvis_bounds =
					static_cast<const OMPLFullBodyStateSpace*>(space_)->getPelvisPositionBounds();
			const double &pelvis_rot =
					static_cast<const OMPLFullBodyStateSpace*>(space_)->getPelvisRotationBound();
			rstate->pelvisPose().setXYZ(rng_.uniformReal(pelvis_bounds.low[0], pelvis_bounds.high[0]), rng_.uniformReal(pelvis_bounds.low[1], pelvis_bounds.high[1]), rng_.uniformReal(pelvis_bounds.low[2], pelvis_bounds.high[2]));

			rstate->pelvisPose().rotation().setAxisAngle(0, 0, 1, rng_.uniformReal(0, pelvis_rot));
		}
	}

	void OMPLFullBodyStateSampler::sampleUniformNear(ob::State *state, const ob::State *near,
			const double distance)
	{
//		if (distance >= .25 * boost::math::constants::pi<double>())
//		{
//			sampleUniform(state);
//			return;
//		}
//		double d = rng_.uniform01();
//		OMPLFullBodyStateSpace::StateType *qs =
//				static_cast<OMPLFullBodyStateSpace::StateType*>(state), *qn =
//				static_cast<OMPLFullBodyStateSpace::StateType*>(near);

		WARNING_NAMED("OMPLFullBodyStateSampler", "sampleUniformNear not implemented");
	}

	void OMPLFullBodyStateSampler::sampleGaussian(ob::State *state, const ob::State * mean,
			const double stdDev)
	{
		WARNING_NAMED("OMPLFullBodyStateSampler", "sampleGaussian not implemented");
	}

	EReturn OMPLFullBodyStateSpace::OMPLStateToEigen(const ob::State *ompl, Eigen::VectorXd &eigen)
	{
		if (!ompl)
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		eigen.resize(fullbody_ ? upperbody_dim_ + 7 : upperbody_dim_);
		memcpy(eigen.segment(0, upperbody_dim_).data(), ompl->as<OMPLFullBodyStateSpace::StateType>()->upperBodyConfiguration().values, sizeof(double)
				* upperbody_dim_);
		if (fullbody_)
		{
			eigen(upperbody_dim_) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().getX();
			eigen(upperbody_dim_ + 1) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().getY();
			eigen(upperbody_dim_ + 2) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().getZ();
			eigen(upperbody_dim_ + 3) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().x;
			eigen(upperbody_dim_ + 4) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().y;
			eigen(upperbody_dim_ + 5) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().z;
			eigen(upperbody_dim_ + 6) =
					ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().w;
		}
		return SUCCESS;
	}

	EReturn OMPLFullBodyStateSpace::EigenToOMPLState(const Eigen::VectorXd &eigen, ob::State *ompl)
	{
		if (!fullbody_ && eigen.rows() < upperbody_dim_)
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		if (fullbody_ && eigen.rows() != upperbody_dim_ + 7)
		{
			INDICATE_FAILURE
			return FAILURE;
		}
		if (!ompl)
		{
			INDICATE_FAILURE
			return FAILURE;
		}

		memcpy(ompl->as<OMPLFullBodyStateSpace::StateType>()->upperBodyConfiguration().values, eigen.segment(0, upperbody_dim_).data(), sizeof(double)
				* upperbody_dim_);
		if (fullbody_)
		{
			ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().setXYZ(eigen(upperbody_dim_), eigen(upperbody_dim_
					+ 1), eigen(upperbody_dim_ + 2));
			ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().x =
					eigen(upperbody_dim_ + 3);
			ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().y =
					eigen(upperbody_dim_ + 4);
			ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().z =
					eigen(upperbody_dim_ + 5);
			ompl->as<OMPLFullBodyStateSpace::StateType>()->pelvisPose().rotation().w =
					eigen(upperbody_dim_ + 6);
		}
		return SUCCESS;
	}
}

