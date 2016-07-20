/*
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

#ifndef JOINTLIMIT_H_
#define JOINTLIMIT_H_

#include <exotica/TaskMap.h>
#include <exotica/Factory.h>
#include <exotica/Test.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

namespace exotica
{
  /**
   * \brief	Implementation of joint limits task map.
   * 			Note: we dont want to always stay at the centre of the joint range,
   * 			be lazy as long as the joint is not too close to the low/high limits
   */
  class JointLimit: public TaskMap
  {
    public:
      //	Default constructor
      JointLimit();
      //	Default destructor
      ~JointLimit();

      /**
       * @brief	Concrete implementation of update method
       * @param	x	Joint space configuration
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t);

      /**
       * @brief	Get the task space dimension
       * @return	Exotica return type, SUCCESS if succeeded
       */
      virtual void taskSpaceDim(int & task_dim);

    protected:
      /**
       * @brief	Concrete implementation of initialisation from xml
       * @param	handle	XML handler
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

    private:
      Eigen::VectorXd low_limits_;	//	Lower joint limits
      Eigen::VectorXd high_limits_;	//	Higher joint limits
      Eigen::VectorXd center_;		//	Center of the joint range
      Eigen::VectorXd tau_;			//	Joint limits tolerance
      bool initialised_;

  };
}

#endif /* JOINTLIMIT_H_ */
