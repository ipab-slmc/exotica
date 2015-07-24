/*
 * DMeshProjection.h
 *
 *  Created on: 8 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_PROJECTIONS_DMESHPROJECTION_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_PROJECTIONS_DMESHPROJECTION_H_
#include <ompl/base/ProjectionEvaluator.h>
#include "exotica/Scene.h"
#include "ompl_solver/OMPLStateSpace.h"
#include "ompl_solver/OMPLSE3RNCompoundStateSpace.h"

namespace exotica
{
	class DMeshProjection: public ompl::base::ProjectionEvaluator
	{
		public:
			DMeshProjection(const ompl::base::StateSpacePtr &space,
					const std::vector<std::string> & links,
					std::vector<std::pair<int, int> > dist_index, Scene_ptr & scene) :
							ompl::base::ProjectionEvaluator(space),
							links_(links),
							dist_index_(dist_index),
							scene_(scene),space_(space)
			{
				std::string str=space_->getName();
				if(str.compare("OMPLSE3RNCompoundStateSpace")==0)
					compound_=true;
				else
					compound_=false;
			}

			~DMeshProjection()
			{
				//TODO
			}

			virtual unsigned int getDimension(void) const
			{
				return dist_index_.size();
			}

			virtual void defaultCellSizes()
			{
				cellSizes_.clear();
				cellSizes_.resize(dist_index_.size(), 0.1);
			}

			virtual void project(const ompl::base::State *state,
					ompl::base::EuclideanProjection &projection) const
			{
//				HIGHLIGHT_NAMED("DMeshProjection", "Calling project");
				Eigen::VectorXd q(space_->getDimension());
				compound_? space_->as<OMPLSE3RNCompoundStateSpace>()->OMPLStateToEigen(state,q): space_->as<OMPLStateSpace>()->copyFromOMPLState(state,q);
				scene_->update(q, 0);
				std::vector<KDL::Frame> poses;
				scene_->getPoses(links_, poses);
				for (std::size_t i = 0; i < dist_index_.size(); ++i)
					projection(i) =(poses[dist_index_[i].second].p-poses[dist_index_[i].first].p).Norm();
			}

		private:
			std::vector<std::string> links_;
			std::vector<std::pair<int, int> > dist_index_;
			Scene_ptr scene_;
			boost::shared_ptr<ob::StateSpace> space_;
			bool compound_;
	};
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_PROJECTIONS_DMESHPROJECTION_H_ */
