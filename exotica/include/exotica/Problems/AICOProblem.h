/*
 *  Created on: 19 Apr 2014
 *      Author: Vladimir Ivan
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

/** \file AICOProblem.h
 \brief Approximate Inference Control Problem specification */

#ifndef AICOPROBLEM_H_
#define AICOPROBLEM_H_

#include <exotica/PlanningProblem.h>
#include "exotica/Definitions/TaskSqrError.h"
#include <exotica/AICOProblemInitializer.h>

namespace exotica
{

  /**
   * \brief Problem specification for Approximate Inference Control method.
   * \ingroup AICO
   */
  class AICOProblem: public PlanningProblem, public Instantiable<AICOProblemInitializer>
  {
    public:
      AICOProblem();
      virtual ~AICOProblem();

      virtual void Instantiate(AICOProblemInitializer& init);

      /**
       * \brief Get number of time steps
       * @return Number of time steps
       */
      virtual int getT();

      /**
       * \brief Set number of time steps
       */
      void setTime(int T);

      /**
       * \brief Get number of time steps
       * @param T_ Number of time steps to return
       */
      void getT(int& T_);

      /**
       * \brief Get time step duration
       * @return Time step duration
       */
      virtual double getTau();

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

      /**
       * @brief update Updates the AICO problem, adding lazy update of task maps for which the task definition has rho=0.
       * @param x State
       * @param t time step
       * @return  Indication of success TODO
       */
      void update(Eigen::VectorXdRefConst x, const int t);

      virtual void reinitialise(rapidjson::Document& document,
          boost::shared_ptr<PlanningProblem> problem);

    protected:
      /**
       * \brief Derived Initialiser (from XML): PURE VIRTUAL
       * @param handle The handle to the XML-element describing the Problem Definition
       * @return Indication of success/failure
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);
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
