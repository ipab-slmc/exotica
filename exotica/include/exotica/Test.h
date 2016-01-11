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

#ifndef EXOTICA_TEST_FACTORY_H
#define EXOTICA_TEST_FACTORY_H

#include <boost/shared_ptr.hpp>
#include <map>

#include "exotica/Tools.h"

/**
 * \brief Generic Factory Macro definition: to be specialsed by each new test/identifier type
 * @param IDENT   Identifier Type   : To be specialised
 * @param TOBJT   Test Object Type  : To be specialised
 * @param TYPE    The name to identify the class (should be of type IDENT)
 * @param TEST    The Test object (of type TOBJT)
 */
#define EXOTICA_REGISTER_TEST(IDENT, TOBJT, TYPE, TEST) static exotica::TestRegistrar<IDENT, TOBJT> EX_UNIQ(test_registrar_, __LINE__) (TYPE, TEST);

/**
 * \brief A specialisation of the above macro where both the identifier and the test object are string (specifically the TEST is the path to the xml-file)
 */
#define REGISTER_FOR_XML_TEST(TYPE, TEST) EXOTICA_REGISTER_TEST(std::string, std::string, TYPE, TEST)

namespace exotica
{
  /**
   * \brief Templated Singleton Test Factory
   * @param I   The identifier type (typically a string: should be the same as used to register the object type)
   * @param T   The Test object type (typically a string for a filename)
   */
  template<typename I, typename T>
  class Test: public Object
  {
    public:
      /**
       * \brief Singleton implementation: returns a reference to a singleton instance of the instantiated class
       */
      static Test<I, T> & Instance(void)
      {
        static Test<I, T> tester_; //!< Declared static so will only be created once
        return tester_;     //!< At other times, just return the reference to it
      }
      ;

      /**
       * \brief Registers a new test
       * @param type[in]  The name of the class (typically string): must be a unique identifier
       * @param test[in]  The test class
       * @return          Indication of success: Returns SUCCESS if registered, or PAR_ERR if the type already contains a registered test
       */
      EReturn registerTest(const I & type, const T & test)
      {
        if (test_registry_.find(type) == test_registry_.end()) //!< If it does not already exist
        {
          test_registry_[type] = test;
          return SUCCESS;
        }
        else //!< I.e. it exists, then cannot re-create it!
        {
          return PAR_ERR;
        }
      }
      ;

      /**
       * \brief Finds the testing object: must implement the copy operator
       * @param type[in]   Identifier as used by the instantiation of the factory
       * @param test[out]  Placeholder for a copy of the test object
       * @return           Indication of success: SUCCESS if ok and PAR_ERR if the type is not found
       */
      EReturn getTest(const I & type, T & test)
      {
        auto it = test_registry_.find(type);  //!< Attempt to find it
        if (it != test_registry_.end())       //!< If exists
        {
          test = it->second;                  //!< Copy the element
          return SUCCESS;
        }
        else
        {
          return PAR_ERR;   //!< Type not found
        }
      }
      ;

    private:
      /**
       * \brief Private Constructor 
       */
      inline explicit Test<I, T>()
      {
      }
      ;

      /** The Map containing the register of the different types of classes **/
      std::map<I, T> test_registry_;
  };

  /**
   * \brief Registration Class for the object type: Also templated:
   * @param I   The Identifier type (typically string)
   * @param T   The Test object type (typically string indicating filename)
   */
  template<typename I, typename T>
  class TestRegistrar
  {
    public:
      /**
       * \brief Public Constructor which is only used to register the test
       * @param name   The name for the new class type
       * @param test   The test object
       */
      TestRegistrar(const I & name, const T & test)
      {
        Test<I, T>::Instance().registerTest(name, test);
      }
      ;
  };

  /**
   * \brief Typedefinition of the specialisation for the XML-based tests
   */
  typedef Test<std::string, std::string> XMLTester;
}

#endif
