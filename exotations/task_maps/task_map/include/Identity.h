/*
 *      Author: Michael Camilleri
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

#ifndef EXOTICA_GENERIC_IDENTITY_H
#define EXOTICA_GENERIC_IDENTITY_H

#include <exotica/TaskMap.h>
#include <task_map/IdentityInitializer.h>

namespace exotica
{
  class Identity: public TaskMap, public Instantiable<IdentityInitializer>
  {
    public:
      /**
       * \brief Default constructor
       */
      Identity();
      virtual ~Identity()
      {
      }

      virtual void Instantiate(IdentityInitializer& init);

      /**
       * \brief Concrete implementation of the update method
       */
      virtual void update(Eigen::VectorXdRefConst& x, const int t);

      /**
       * \brief Concrete implementation of the task-space size
       */
      virtual void taskSpaceDim(int & task_dim);

      void initialise(std::string& postureName,
          std::vector<std::string>& joints, bool skipUnknown = false);
      virtual void initialise(const rapidjson::Value& a);

      bool useRef;
      std::vector<int> jointMap;
      Eigen::VectorXd jointRef;

    protected:
      /**
       * \brief Concrete implementation of the initialisation method
       */
      virtual void initDerived(tinyxml2::XMLHandle & handle);

      int getJointIDexternal(std::string& name);
      int getJointID(std::string& name);

  };

  typedef boost::shared_ptr<exotica::Identity> Identity_ptr;
}
#endif
