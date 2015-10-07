/*
 * FRRTConnect.h
 *
 *  Created on: 5 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTCONNECT_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTCONNECT_H_

#include "FlexiblePlanner.h"

namespace ompl
{
  namespace geometric
  {
    ///	\brief	The bi-directional FRRT
    class FRRTConnect: public FlexiblePlanner
    {
      public:
        /*
         * \brief	Constructor
         * @param	si	Space information
         */
        FRRTConnect(const base::SpaceInformationPtr &si);

        /*
         * \brief	Destructor
         */
        virtual ~FRRTConnect();

        virtual void getPlannerData(base::PlannerData &data) const;
        virtual base::PlannerStatus solve(
            const base::PlannerTerminationCondition &ptc);
        virtual void clear();

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
        virtual void setup();
      protected:
        class FM_RRTConnect: public FlexibleMotion
        {
          public:
            /*
             * \brief	Constructor
             */
            FM_RRTConnect()
                : FlexibleMotion(), root(NULL), parent(NULL)
            {
            }

            FM_RRTConnect(const base::SpaceInformationPtr &si)
                : FlexibleMotion(si), root(NULL), parent(NULL)
            {
            }
            /*
             * \brief	Destructor
             */
            ~FM_RRTConnect()
            {

            }
            FM_RRTConnect *parent;
            const base::State *root;
        };
        /** \brief Set a different nearest neighbors datastructure */
        template<template<typename T> class NN>
        void setNearestNeighbors()
        {
          tStart_.reset(new NN<FM_RRTConnect*>());
          tGoal_.reset(new NN<FM_RRTConnect*>());
        }
        /** \brief A nearest-neighbor datastructure representing a tree of motions */
        typedef boost::shared_ptr<NearestNeighbors<FM_RRTConnect*> > TreeData;

        /** \brief Information attached to growing a tree of motions (used internally) */
        struct TreeGrowingInfo
        {
            base::State *xstate;
            FM_RRTConnect *xmotion;
            bool start;
        };

        /** \brief The state of the tree after an attempt to extend it */
        enum GrowState
        {
          /// no progress has been made
          TRAPPED,
          /// progress has been made towards the randomly sampled state
          ADVANCED,
          /// the randomly sampled state was reached
          REACHED
        };

        /** \brief Free the memory allocated by this planner */
        void freeMemory();

        /** \brief Grow a tree towards a random state */
        GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi,
            FM_RRTConnect *rmotion);

        /** \brief The start tree */
        TreeData tStart_;

        /** \brief The goal tree */
        TreeData tGoal_;

        /** \brief The maximum length of a motion to be added to a tree */
        double maxDistance_;

        /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
        std::pair<base::State*, base::State*> connectionPoint_;
      private:
        bool localSolve(FM_RRTConnect *sm, base::State *is, FM_RRTConnect *gm);
    };
  }	//	Geometric namespace
}	//	OMPL namespace

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FRRTCONNECT_H_ */
