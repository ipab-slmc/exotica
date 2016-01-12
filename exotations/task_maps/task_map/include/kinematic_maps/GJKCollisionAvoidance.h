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

#ifndef EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_GJKCOLLISIONAVOIDANCE_H_
#define EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_GJKCOLLISIONAVOIDANCE_H_

#include "CollisionAvoidance.h"
namespace exotica
{
  /*
   * \brief	This class uses GilbertJohnson-Keerthi algorithm(GJK) algorithms to calculate the
   * 			distance information between two objects
   * 			A fast procedure for computing the distance between complex objects in three-dimensional space
   * 			http://graphics.stanford.edu/courses/cs448b-00-winter/papers/gilbert.pdf
   */
  class GJKCollisionAvoidance: public CollisionAvoidance
  {
    public:
      GJKCollisionAvoidance();
      virtual ~GJKCollisionAvoidance();

      ///	We only need to change the update function, other functions state the same as in collision avoidance
      virtual EReturn update(Eigen::VectorXdRefConst x, const int t);
  };
}

#endif /* EXOTICA_EXOTATIONS_TASK_MAPS_TASK_MAP_INCLUDE_KINEMATIC_MAPS_GJKCOLLISIONAVOIDANCE_H_ */
