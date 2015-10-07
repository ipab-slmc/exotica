/*
 * FRRT.h
 *
 *  Created on: 22 Apr 2015
 *      Author: yiming
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
