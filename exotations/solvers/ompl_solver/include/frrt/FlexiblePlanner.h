/*
 * FlexiblePlanner.h
 *
 *  Created on: 8 Jun 2015
 *      Author: yiming
 */

#ifndef EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FLEXIBLEPLANNER_H_
#define EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FLEXIBLEPLANNER_H_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "exotica/EXOTica.hpp"
#include <ompl_solver/OMPLGoalSampler.h>
#include <ik_solver/ik_solver.h>
namespace ompl
{
  namespace geometric
  {
    /*
     * \brief	Implementation of FlexiblePlanner base, mainly designed to manage the local solver in a systematic way.
     * 			So that the actual algorithms can call 'localSolve' directly
     */
    class FlexiblePlanner: public base::Planner
    {
      public:
        /*
         * \brief	Default constructor
         * @param	si		OMPL space information
         * @param	name	Planner name
         */
        FlexiblePlanner(const base::SpaceInformationPtr &si,
            const std::string & name);

        /*
         * \brief	Default destructor
         */
        virtual ~FlexiblePlanner();

        /*
         * \brief	Set up the local planner (EXOTica)
         * @param	xml_file	XML configuration file
         * @param	scene		EXOTica scene
         * @return	True if succeeded, false otherwise
         */
        bool setUpLocalPlanner(const std::string & xml_file,
            const exotica::Scene_ptr & scene);

        bool resetSceneAndGoal(const exotica::Scene_ptr & scene,
            const Eigen::VectorXd & goal);

        ///	State sampler
        base::StateSamplerPtr sampler_;
        ///	Random number generator
        RNG rng_;

        int checkCnt_;
      protected:
        bool copyStateToEigen(const ob::State *state, Eigen::VectorXd & eigen)
        {
          std::string spacename = si_->getStateSpace()->getName();
          if (spacename.compare("OMPLSE3RNCompoundStateSpace") == 0)
          {
            return exotica::ok(
                si_->getStateSpace()->as<exotica::OMPLSE3RNCompoundStateSpace>()->OMPLStateToEigen(
                    state, eigen));
          }
          else if (spacename.compare("OMPLStateSpace") == 0)
          {
            return exotica::ok(
                si_->getStateSpace()->as<exotica::OMPLStateSpace>()->copyFromOMPLState(
                    state, eigen));
          }
          else
          {
            ERROR("Can not convert state space "<<spacename);
          }
          return false;
        }
        bool copyEigenToState(const Eigen::VectorXd & eigen, ob::State *state)
        {
          std::string spacename = si_->getStateSpace()->getName();
          if (spacename.compare("OMPLSE3RNCompoundStateSpace") == 0)
          {
            return exotica::ok(
                si_->getStateSpace()->as<exotica::OMPLSE3RNCompoundStateSpace>()->EigenToOMPLState(
                    eigen, state));
          }
          else if (spacename.compare("OMPLStateSpace") == 0)
          {
            return exotica::ok(
                si_->getStateSpace()->as<exotica::OMPLStateSpace>()->copyToOMPLState(
                    state, eigen));
          }
          else
          {
            ERROR("Can not convert state space "<<spacename);
          }
          return false;
        }
        class FlexibleMotion
        {
          public:
            /*
             * \brief	Constructor
             */
            FlexibleMotion()
                : state(NULL), inter_state(NULL)
            {
            }

            FlexibleMotion(const base::SpaceInformationPtr &si)
                : state(si->allocState()), inter_state(NULL)
            {
            }
            /*
             * \brief	Destructor
             */
            ~FlexibleMotion()
            {

            }

            ///	The OMPL state
            base::State *state;
            ///	Internal state
            base::State *inter_state;
            ///	The internal flexible path
            boost::shared_ptr<Eigen::MatrixXd> internal_path;
        };

        /*
         * \brief	Compute distance between motions (actually distance between contained states)
         * @param	a		Motion a
         * @param	b		Motion b
         * @return	Distance between a and b
         */
        double distanceFunction(const FlexibleMotion *a,
            const FlexibleMotion *b) const
        {
          double d = si_->distance(a->state, b->state);
          return d;
        }
        ///	Local solver
        exotica::EReturn localSolve(const Eigen::VectorXd & qs,
            Eigen::VectorXd & qg, Eigen::MatrixXd & solution);
        exotica::Server_ptr ser_;
        exotica::IKsolver_ptr local_solver_;
        boost::shared_ptr<exotica::Identity> local_map_;
        exotica::EParam<std_msgs::Float64> gTau_;
        exotica::EParam<std_msgs::Float64> lTau_;
        Eigen::VectorXd global_goal_;
    };

  }
}

#endif /* EXOTICA_EXOTATIONS_SOLVERS_OMPL_SOLVER_INCLUDE_FRRT_FLEXIBLEPLANNER_H_ */
