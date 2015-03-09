/*
 * AICOProblem.h
 *
 *  Created on: 23 Apr 2014
 *      Author: Vladimir Ivan
 */

/** \file AICOProblem.h
 \brief Approximate Inference Control Problem specification */

#ifndef AICOPROBLEM_H_
#define AICOPROBLEM_H_

#include <exotica/PlanningProblem.h>

namespace exotica
{

	/**
	 * \brief Problem specification for Approximate Inference Control method.
	 * \ingroup AICO
	 */
	class AICOProblem: public PlanningProblem
	{
		public:
			AICOProblem();
			virtual ~AICOProblem();

			/**
			 * \brief Get number of time steps
			 * @return Number of time steps
			 */
			int getT();

			/**
			 * \brief Set number of time steps
			 */
			EReturn setTime(int T);

			/**
			 * \brief Get number of time steps
			 * @param T_ Number of time steps to return
			 */
			void getT(int& T_);

			/**
			 * \brief Get time step duration
			 * @return Time step duration
			 */
			double getTau();

			/**
			 * \brief Get time step duration
			 * @param tau_ Time step duration to return
			 */
			void getTau(double& tau_);

			/**
			 * \brief Get trajectory duration
			 * @return Trajectory duration
			 */
			double getDuration();

			/**
			 * \brief Returns the reference to the task definition map.
			 * @return Task definitions
			 */
			TaskDefinition_map& getTaskDefinitions();

			/**
			 * \brief	Get task maps
			 * @return Task maps
			 */
			TaskMap_map& getTaskMaps();

			/**
			 * \brief Get kinematic system transition error covariance
			 * @return Kinematic system transition error covariance
			 */
			Eigen::MatrixXd getW();

			/**
			 * \brief Get system transition error covariance multipler
			 * @return Transition error covariance multipler
			 */
			double getQrate();

			/**
			 * \brief Get kinematic system transition error covariance multiplier
			 * @return Kinematic system transition error covariance multiplier
			 */
			double getWrate();

			/**
			 * \brief Get control error covariance multipler
			 * @return Control error covariance multipler
			 */
			double getHrate();

		protected:
			/**
			 * \brief Derived Initialiser (from XML): PURE VIRTUAL
			 * @param handle The handle to the XML-element describing the Problem Definition
			 * @return Indication of success/failure
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle);
		private:
			int T; //!< Number of time steps
			double tau; //!< Time step duration
			Eigen::MatrixXd W; //!< Kinematic system transition error covariance (constant throughout the trajectory)
			double Q_rate; //!< System transition error covariance multipler (per unit time) (constant throughout the trajectory)
			double H_rate; //!< Control error covariance multipler (per unit time) (constant throughout the trajectory)
			double W_rate; //!< Kinematic system transition error covariance multiplier (constant throughout the trajectory)

	};

	typedef boost::shared_ptr<exotica::AICOProblem> AICOProblem_ptr;
} /* namespace exotica */

#endif /* AICOPROBLEM_H_ */
