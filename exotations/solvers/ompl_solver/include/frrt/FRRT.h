/*
 *  Created on: 22 Apr 2015
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

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRT_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRT_H_

#include "FlexiblePlanner.h"
namespace ompl
{
  namespace geometric
  {
    /*
     * \brief	Implementation of Flexible RRT Planning Algorithm
     */
    class FRRT: public FlexiblePlanner
    {
      public:
        /*
         * \brief	Default constructor
         * @param	si		OMPL space information
         */
        FRRT(const base::SpaceInformationPtr &si);

        /*
         * \brief	Default destructor
         */
        virtual ~FRRT();

        /*
         * \brief	Get the planning data
         * @param	data	Planning data
         */
        virtual void getPlannerData(base::PlannerData &data) const;

        /*
         * \brief	Solve the planning problem
         * @param	ptc		Termination condition
         * @return	OMPL planning status
         */
        virtual base::PlannerStatus solve(
            const base::PlannerTerminationCondition &ptc);

        /*
         * \brief	Clear planner information
         */
        virtual void clear();

        /*
         * \brief	Set goal bias value
         * @param	bias	Goal bias
         */
        void setGoalBias(double bias)
        {
          goalBias_ = bias;
        }

        /*
         * \brief	Get goal bias value
         * @return	Goal bias
         */
        double getGoalBias() const
        {
          return goalBias_;
        }

        /*
         * \brief	Set maximum distance
         * @param	dist	Maximum distance
         */
        void setRange(double dist)
        {
          maxDist_ = dist;
        }

        /*
         * \brief	Get maximum distance
         * @return	Maximum distance
         */
        double getRange() const
        {
          return maxDist_;
        }

        /*
         * \brief	Set up the planner
         */
        virtual void setup();

      protected:
        class FM_RRT: public FlexibleMotion
        {
          public:
            /*
             * \brief	Constructor
             */
            FM_RRT()
                : FlexibleMotion(), parent(NULL)
            {
            }

            FM_RRT(const base::SpaceInformationPtr &si)
                : FlexibleMotion(si), parent(NULL)
            {
            }
            /*
             * \brief	Destructor
             */
            ~FM_RRT()
            {

            }
            FM_RRT *parent;
        };
        template<template<typename T> class NN>
        void setNearestNeighbors()
        {
          nn_.reset(new NN<FM_RRT*>());
        }
        /*
         * \brief	Release memory
         */
        void freeMemory();

        ///	The tree
        boost::shared_ptr<NearestNeighbors<FM_RRT*> > nn_;
        /// Goal bias
        double goalBias_;
        ///	Maximum distance
        double maxDist_;
        ///	Random number generator
        ///	Last goal
        FlexibleMotion *lastGoalMotion_;

      private:
        ///	Local solver
        bool localSolve(FM_RRT *sm, base::State *is, FM_RRT *gm);

        ///	For analyse
        std::vector<int> try_cnt_;
        std::vector<int> suc_cnt_;
    };
  }
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRT_H_ */
