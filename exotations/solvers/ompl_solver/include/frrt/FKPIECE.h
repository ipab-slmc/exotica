/*
 *  Created on: 12 Jun 2015
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FKPIECE_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FKPIECE_H_
#include <ompl/geometric/planners/kpiece/Discretization.h>
#include "frrt/FlexiblePlanner.h"
namespace ompl
{

  namespace geometric
  {
    /** \brief Kinematic Planning by Interior-Exterior Cell Exploration */
    class FKPIECE: public FlexiblePlanner
    {
      public:

        /** \brief Constructor */
        FKPIECE(const base::SpaceInformationPtr &si);

        virtual ~FKPIECE();

        virtual base::PlannerStatus solve(
            const base::PlannerTerminationCondition &ptc);

        virtual void clear();

        /** \brief Set the goal bias.

         In the process of randomly selecting states in the state
         space to attempt to go towards, the algorithm may in fact
         choose the actual goal state, if it knows it, with some
         probability. This probability is a real number between 0.0
         and 1.0; its value should usually be around 0.05 and
         should not be too large. It is probably a good idea to use
         the default value. */
        void setGoalBias(double goalBias)
        {
          goalBias_ = goalBias;
        }

        /** \brief Get the goal bias the planner is using */
        double getGoalBias() const
        {
          return goalBias_;
        }

        /** \brief Set the range the planner is supposed to use.

         This parameter greatly influences the runtime of the
         algorithm. It represents the maximum length of a
         motion to be added in the tree of motions. */
        void setRange(double distance)
        {
          maxDistance_ = distance;
        }

        /** \brief Get the range the planner is using */
        double getRange() const
        {
          return maxDistance_;
        }

        /** \brief Set the fraction of time for focusing on the
         border (between 0 and 1). This is the minimum fraction
         used to select cells that are exterior (minimum
         because if 95% of cells are on the border, they will
         be selected with 95% chance, even if this fraction is
         set to 90%)*/
        void setBorderFraction(double bp)
        {
          disc_.setBorderFraction(bp);
        }

        /** \brief Get the fraction of time to focus exploration
         on boundary */
        double getBorderFraction() const
        {
          return disc_.getBorderFraction();
        }

        /** \brief When extending a motion, the planner can decide
         to keep the first valid part of it, even if invalid
         states are found, as long as the valid part represents
         a sufficiently large fraction from the original
         motion. This function sets the minimum acceptable
         fraction (between 0 and 1). */
        void setMinValidPathFraction(double fraction)
        {
          minValidPathFraction_ = fraction;
        }

        /** \brief Get the value of the fraction set by setMinValidPathFraction() */
        double getMinValidPathFraction() const
        {
          return minValidPathFraction_;
        }

        /** \brief When extending a motion from a cell, the
         extension can be successful or it can fail. If the
         extension fails, the score of the cell is multiplied
         by \e factor. These number should be in the range (0, 1]. */
        void setFailedExpansionCellScoreFactor(double factor)
        {
          failedExpansionScoreFactor_ = factor;
        }

        /** \brief Get the factor that is multiplied to a cell's
         score if extending a motion from that cell failed. */
        double getFailedExpansionCellScoreFactor() const
        {
          return failedExpansionScoreFactor_;
        }

        /** \brief Set the projection evaluator. This class is
         able to compute the projection of a given state. */
        void setProjectionEvaluator(
            const base::ProjectionEvaluatorPtr &projectionEvaluator)
        {
          projectionEvaluator_ = projectionEvaluator;
        }

        /** \brief Set the projection evaluator (select one from
         the ones registered with the state space). */
        void setProjectionEvaluator(const std::string &name)
        {
          projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
        }

        /** \brief Get the projection evaluator */
        const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
        {
          return projectionEvaluator_;
        }

        virtual void setup();

        virtual void getPlannerData(base::PlannerData &data) const;

      protected:

        /** \brief Representation of a motion for this algorithm */
        class Motion
        {
          public:

            Motion()
                : state(NULL), parent(NULL), inter_state(NULL)
            {
            }

            /** \brief Constructor that allocates memory for the state */
            Motion(const base::SpaceInformationPtr &si)
                : state(si->allocState()), parent(NULL), inter_state(NULL)
            {
            }

            ~Motion()
            {
            }

            ///	The OMPL state
            base::State *state;
            ///	Internal state
            base::State *inter_state;
            ///	The parent node
            Motion *parent;
            ///	The internal flexible path
            boost::shared_ptr<Eigen::MatrixXd> internal_path;
        };

        /** \brief Free the memory for a motion */
        void freeMotion(Motion *motion);

        /** \brief The tree datastructure and the grid that covers it */
        Discretization<Motion> disc_;

        /** \brief This algorithm uses a discretization (a grid)
         to guide the exploration. The exploration is imposed
         on a projection of the state space. */
        base::ProjectionEvaluatorPtr projectionEvaluator_;

        /** \brief When extending a motion from a cell, the
         extension can fail. If it is, the score of the cell is
         multiplied by this factor. */
        double failedExpansionScoreFactor_;

        /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
        double goalBias_;

        /** \brief When extending a motion, the planner can decide
         to keep the first valid part of it, even if invalid
         states are found, as long as the valid part represents
         a sufficiently large fraction from the original
         motion */
        double minValidPathFraction_;

        /** \brief The maximum length of a motion to be added to a tree */
        double maxDistance_;

        /** \brief The most recent goal motion.  Used for PlannerData computation */
        Motion *lastGoalMotion_;

      private:
        bool localSolve(Motion *sm, base::State *is, Motion *gm);
    };

  }
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FKPIECE_H_ */
