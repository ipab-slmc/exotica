/*
 *  Created on: 15 Jul 2014
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

#ifndef IK_PROBLEM_H_
#define IK_PROBLEM_H_
#include <exotica/PlanningProblem.h>
#include "task_definition/TaskSqrError.h"

namespace exotica
{
  /**
   * IK problem implementation
   */
  class IKProblem: public PlanningProblem
  {
    public:
      IKProblem();
      virtual ~IKProblem();

      /**
       * \brief	Get configuration weight
       * @return	configuration weight
       */
      Eigen::MatrixXd getW();

      int getT();

      /**
       * \brief	Get tolerance
       * @return	tolerance
       */
      double getTau();
      void setTau(double tau);

      virtual void reinitialise(rapidjson::Document& document,
          boost::shared_ptr<PlanningProblem> problem);
    protected:
      /**
       * \brief Derived Initialiser (from XML): PURE VIRTUAL
       * @param handle The handle to the XML-element describing the Problem Definition
       * @return Indication of success/failure
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

      Eigen::MatrixXd config_w_;	//Configuration weight
      double tau_;	// Tolerance
      int T_;

  };
  typedef boost::shared_ptr<exotica::IKProblem> IKProblem_ptr;
}

#endif /* IK_PROBLEM_H_ */
