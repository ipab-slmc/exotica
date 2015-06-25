/*
 * OMPLFullBodyStateSpace.h
 *
 *  Created on: 22 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLFULLBODYSTATESPACE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLFULLBODYSTATESPACE_H_

#include "exotica/EXOTica.hpp"
#include "ompl_solver/OMPLProblem.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ob = ompl::base;
namespace exotica
{

	class OMPLFullBodyStateSampler: public ob::StateSampler
	{
		public:
			OMPLFullBodyStateSampler(const ob::StateSpace *space) :
					ob::StateSampler(space)
			{
			}
			virtual void sampleUniform(ob::State *state);
			virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance);
			virtual void sampleGaussian(ob::State *state, const ob::State * mean, const double stdDev);
	};
	class OMPLFullBodyStateSpace: public ompl::base::CompoundStateSpace
	{
		public:
			class StateType: public ob::CompoundStateSpace::StateType
			{
				public:
					StateType() :
							CompoundStateSpace::StateType()
					{

					}

					const ob::RealVectorStateSpace::StateType & upperBodyConfiguration() const
					{
						return *as<ob::RealVectorStateSpace::StateType>(0);
					}

					ob::RealVectorStateSpace::StateType & upperBodyConfiguration()
					{
						return *as<ob::RealVectorStateSpace::StateType>(0);
					}

					const ob::SE3StateSpace::StateType & pelvisPose() const
					{
						return *as<ob::SE3StateSpace::StateType>(1);
					}
					ob::SE3StateSpace::StateType & pelvisPose()
					{
						return *as<ob::SE3StateSpace::StateType>(1);
					}
			};

			OMPLFullBodyStateSpace(unsigned int dim, bool fullbody);
			virtual ~OMPLFullBodyStateSpace();
			virtual unsigned int getDimension() const;
			virtual ompl::base::StateSamplerPtr allocDefaultStateSampler();

			static boost::shared_ptr<OMPLFullBodyStateSpace> FromProblem(OMPLProblem_ptr prob);
			EReturn OMPLStateToEigen(const ob::State *ompl, Eigen::VectorXd &eigen);
			EReturn EigenToOMPLState(const Eigen::VectorXd &eigen, ob::State *ompl);
			/*
			 * \brief	Set the bounds for upper body configuration
			 * @param	bounds		Real vector bounds for upper body
			 */
			void setUpperBodyBounds(const ob::RealVectorBounds &bounds);
			const ob::RealVectorBounds & getUpperBodyBounds() const;

			/*
			 * \brief	Set the bounds for pelvis
			 * @param	xyz			Pelvis XYZ position bounds
			 * @param	dist		Pelvis maximum allowed angle from z-axis
			 */
			void setPelvisBounds(const ob::RealVectorBounds &xyz, const double dist);
			const ob::RealVectorBounds & getPelvisPositionBounds() const;
			const double & getPelvisRotationBound() const;
			bool fullbody_;
		private:
			ob::RealVectorBounds pelvis_xyx_bounds_;
			double pelvis_angle_bound_;
			int upperbody_dim_;
	};
} //	Namespace exotica

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLFULLBODYSTATESPACE_H_ */
