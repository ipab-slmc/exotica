/*
 *  Created on: 8 Jun 2015
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_PROJECTIONS_DMESHPROJECTION_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_PROJECTIONS_DMESHPROJECTION_H_
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl_solver/OMPLSE3RNStateSpace.h>
#include "exotica/Scene.h"
#include "ompl_solver/OMPLStateSpace.h"

namespace exotica
{
  class DMeshProjection: public ompl::base::ProjectionEvaluator
  {
    public:
      DMeshProjection(const ompl::base::StateSpacePtr &space,
          const std::vector<std::string> & links,
          std::vector<std::pair<int, int> > dist_index, Scene_ptr & scene)
          : ompl::base::ProjectionEvaluator(space), links_(links), dist_index_(
              dist_index), scene_(scene), space_(space)
      {
        std::string str = space_->getName();
        if (str.compare("OMPLSE3RNCompoundStateSpace") == 0)
          compound_ = true;
        else
          compound_ = false;
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
        compound_ ?
            space_->as<OMPLSE3RNStateSpace>()->OMPLStateToEigen(state,
                q) :
            space_->as<OMPLStateSpace>()->copyFromOMPLState(state, q);
        scene_->Update(q);
        ArrayFrame poses = scene_->getSolver().Solution->Phi;
        for (std::size_t i = 0; i < dist_index_.size(); ++i)
          projection(i) = (poses(dist_index_[i].second).p
              - poses(dist_index_[i].first).p).Norm();
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
