/*
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

#ifndef EXOTICA_TASKMAP_DISTANCE_H
#define EXOTICA_TASKMAP_DISTANCE_H

#include <exotica/TaskMap.h>
#include <task_map/DistanceInitializer.h>

namespace exotica //!< Since this is part of the core library, it will be within the same namespace
{
  class Distance: public TaskMap, public Instantiable<DistanceInitializer>
  {
    public:
      /**
       * \brief Default constructor
       */
      Distance();
      virtual ~Distance()
      {
      }

       virtual void Instantiate(DistanceInitializer& init);

      /**
       * \brief Concrete implementation of the update method
       */
      virtual void update(Eigen::VectorXdRefConst x, const int t);

      /**
       * \brief Concrete implementation of the task-space size
       */
      virtual void taskSpaceDim(int & task_dim);

      virtual void initialise(const rapidjson::Value& a);

      KDL::Frame ref_pose_;

    protected:
      /**
       * \brief Concrete implementation of TaskMap::initDerived()
       * @return  Always returns success
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);
  };
  typedef boost::shared_ptr<Distance> Distance_Ptr;
}

#endif
