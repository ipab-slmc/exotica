/*
 *  Created on: 12 May 2015
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLPROJECTION_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLPROJECTION_H_

#include <ompl/base/ProjectionEvaluator.h>
#include "ompl_solver/OMPLStateSpace.h"
///	Implementation of OMPL EXOTica Projection (Testing)
namespace exotica
{
class OMPLProjection : public ompl::base::ProjectionEvaluator
{
public:
    OMPLProjection(const ompl::base::StateSpacePtr &space,
                   const std::vector<int> &vars)
        : ompl::base::ProjectionEvaluator(space), variables_(vars)
    {
    }

    ~OMPLProjection()
    {
        //TODO
    }

    virtual unsigned int getDimension(void) const
    {
        return variables_.size();
    }

    virtual void defaultCellSizes()
    {
        cellSizes_.clear();
        cellSizes_.resize(variables_.size(), 0.1);
    }

    virtual void project(const ompl::base::State *state,
                         ompl::base::EuclideanProjection &projection) const
    {
        for (std::size_t i = 0; i < variables_.size(); ++i)
            projection(i) =
                state->as<exotica::OMPLStateSpace::StateType>()->values[variables_[i]];
    }

private:
    std::vector<int> variables_;
};
}
#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLPROJECTION_H_ */
