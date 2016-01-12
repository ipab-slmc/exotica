/*
 *  Created on: 22 Jun 2015
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLSE3RNCOMPOUNDSTATESPACE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLSE3RNCOMPOUNDSTATESPACE_H_

#include "exotica/EXOTica.hpp"
#include "ompl_solver/OMPLProblem.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
namespace ob = ompl::base;
namespace exotica
{
  class OMPLSE3RNCompoundStateSpace: public ompl::base::CompoundStateSpace
  {
    public:
      class StateType: public ob::CompoundStateSpace::StateType
      {
        public:
          StateType()
              : CompoundStateSpace::StateType()
          {

          }

          const ob::RealVectorStateSpace::StateType & RealVectorStateSpace() const
          {
            return *as<ob::RealVectorStateSpace::StateType>(1);
          }

          ob::RealVectorStateSpace::StateType & RealVectorStateSpace()
          {
            return *as<ob::RealVectorStateSpace::StateType>(1);
          }

          const ob::SE3StateSpace::StateType & SE3StateSpace() const
          {
            return *as<ob::SE3StateSpace::StateType>(0);
          }
          ob::SE3StateSpace::StateType & SE3StateSpace()
          {
            return *as<ob::SE3StateSpace::StateType>(0);
          }
      };

      OMPLSE3RNCompoundStateSpace(unsigned int dim, const Server_ptr &server,
          OMPLProblem_ptr &prob);
      virtual ~OMPLSE3RNCompoundStateSpace();
      virtual unsigned int getDimension() const;
      virtual ompl::base::StateSamplerPtr allocDefaultStateSampler();
      static boost::shared_ptr<OMPLSE3RNCompoundStateSpace> FromProblem(
          OMPLProblem_ptr prob, const Server_ptr &server);
      EReturn OMPLStateToEigen(const ob::State *ompl, Eigen::VectorXd &eigen);
      EReturn EigenToOMPLState(const Eigen::VectorXd &eigen, ob::State *ompl);

      /*
       * \brief	Set the bounds for upper body configuration
       * @param	bounds		Real vector bounds for upper body
       */
      void setRealVectorStateSpaceBounds(const ob::RealVectorBounds &bounds);
      const ob::RealVectorBounds & getRealVectorStateSpaceBounds() const;

      /*
       * \brief	Set the bounds for pelvis
       * @param	xyz			Pelvis XYZ position bounds
       * @param	dist		Pelvis maximum allowed angle from z-axis
       */
      void setSE3StateSpaceBounds(const ob::RealVectorBounds &xyz, double dist =
          0);
      const ob::RealVectorBounds & getSE3StateSpaceBounds() const;
      const double getSE3StateSpaceRobotationBound() const;
      void setStart(const Eigen::VectorXd &start);
      void setGoal(const Eigen::VectorXd &goal);
      EParam<exotica::Vector> weights_;
      Eigen::VectorXd start_;
      Eigen::VectorXd goal_;
      double base_dist_;
      EParam<std_msgs::Float64> rn_bias_percentage_;
      ob::RealVectorBounds SO3Bounds_;
      bool useGoal_;
    private:
      Server_ptr server_;
      int realvectordim_;
      OMPLProblem_ptr prob_;
  };
  class OMPLSE3RNCompoundStateSampler: public ob::StateSampler
  {
    public:
      OMPLSE3RNCompoundStateSampler(const ob::StateSpace *space)
          : ob::StateSampler(space)
      {
        EParam<exotica::Vector> weights =
            space->as<OMPLSE3RNCompoundStateSpace>()->weights_;
        weightImportance_.resize(weights->data.size());
        double sum = 0;
        for (int i = 0; i < weights->data.size(); i++)
          sum += weights->data[i];
        for (int i = 0; i < weights->data.size(); i++)
          weightImportance_[i] = weights->data[i] / sum;
      }
      virtual void sampleUniform(ob::State *state);
      virtual void sampleUniformNear(ob::State *state, const ob::State *near,
          const double distance);
      virtual void sampleGaussian(ob::State *state, const ob::State * mean,
          const double stdDev);
      std::vector<double> weightImportance_;
  };
} //	Namespace exotica

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_OMPL_SOLVER_OMPLSE3RNCOMPOUNDSTATESPACE_H_ */
