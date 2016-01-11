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

#ifndef EXOTICA_OBJECT_FACTORY_H
#define EXOTICA_OBJECT_FACTORY_H

#include <boost/shared_ptr.hpp>
#include <map>
#include <typeinfo>

#include "exotica/Server.h"
#include "exotica/Tools.h"

/**
 * \brief Generic Factory Macro definition: to be specialised by each new base type which wishes to make use of a factory for instantiation of derived classes
 * @param IDENT   Identifier Type : should be specialised by redefining a macro
 * @param BASE    Base Object type : should be specialised by redefining a macro
 * @param TYPE    The name to identify the class (should be of type IDENT)
 * @param DERIV   The Derived Class type (should inherit from BASE)
 */
#define EXOTICA_REGISTER(IDENT, BASE, TYPE, DERIV) static exotica::Registrar<IDENT, BASE> EX_UNIQ(object_registrar_, __LINE__) (TYPE, [] () -> BASE * { return new DERIV(); } );

namespace exotica
{
  /**
   * \brief Templated Object factory for Default-constructible classes. The Factory is itself a singleton.
   * @param I   The identifier type (typically a string)
   * @param BO  The Base Object type
   */
  template<typename I, typename BO>
  class Factory: public Object
  {
    public:
      /**
       * \brief Singleton implementation: returns a reference to a singleton instance of the instantiated class
       */
      static Factory<I, BO> & Instance(void)
      {
        static Factory<I, BO> factory_; //!< Declared static so will only be created once
        return factory_;    //!< At other times, just return the reference to it
      }
      ;

      /**
       * \brief Registers a new derived class type
       * @param type[in]    The name of the class (string): must be a unique identifier
       * @param creator[in] A pointer to the creator function
       * @return            Indication of success: Returns SUCCESS if registered, or PAR_ERR if the type already exists
       */
      EReturn registerType(const I & type, BO * (*creator_function)())
      {
        if (type_registry_.find(type) == type_registry_.end()) //!< If it does not already exist
        {
          type_registry_[type] = creator_function;
          return SUCCESS;
        }
        else //!< I.e. it exists, then cannot re-create it!
        {
          return PAR_ERR;
        }
      }
      ;

      /**
       * \brief Lists the valid implementations which are available and registered
       * @param task_types[out] Vector of task-type names
       * @return                Always returns SUCCESS
       */
      EReturn listImplementations(std::vector<I> & registered_types)
      {
        registered_types.clear();
        for (auto it = type_registry_.begin(); it != type_registry_.end(); it++)
        {
          registered_types.push_back(it->first);
        }
        return SUCCESS;
      }
      ;

      /**
       * \brief Creates a new Instance of a derived class
       * @param type  [in]   Identifier as used by the instantiation of the factory
       * @param object[out]  Shared pointer to the object (placeholder)
       * @return             Indication of success: SUCCESS if ok, MEM_ERR if could not create it and PAR_ERR if the type is not found
       */
      EReturn createObject(const I & type, boost::shared_ptr<BO> const & object)
      {
        auto it = type_registry_.find(type);  //!< Attempt to find it
        if (it != type_registry_.end())       //!< If exists
        {
          const_cast<boost::shared_ptr<BO>&>(object).reset(it->second()); //!< Call the function associated with this entry

          if (object != nullptr)
          {
            return SUCCESS;
          }
          else
          {
            ERROR("Object could not be created: pointer = NULL!");
            return MEM_ERR; //!< Memory error
          }
        }
        else
        {
          ERROR("This factory does not recognize type '"<< type << "'");
          return PAR_ERR;   //!< Type not found
        }
      }
      ;

      EReturn createObject(boost::shared_ptr<BO> & object,
          tinyxml2::XMLHandle & handle, const Server_ptr & server)
      {
        if (handle.ToElement())
        {
          if (typeid(I) == typeid(std::string))
          {
            std::string type = std::string(handle.ToElement()->Name());
            auto it = type_registry_.find(type);
            if (it != type_registry_.end())
            {
              const char * atr = handle.ToElement()->Attribute("name");
              if (atr)
              {
                std::string name = std::string(atr);
                if (name.length() > 0)
                {
                  //const_cast< boost::shared_ptr<BO>& >(object).reset(it->second());
                  object.reset(it->second());
                  if (object != nullptr)
                  {
                    //const_cast< boost::shared_ptr<BO>& >(object)->object_name_=name;
                    object->object_name_ = name;
                    object->ns_ = name;
                    return object->initBase(handle, server);
                  }
                  else
                  {
                    ERROR("Object could not be created: pointer = NULL!");
                    return MEM_ERR; //!< Memory error
                  }
                }
                else
                {
                  ERROR(
                      "Object name for object of type '"<< type <<"' was not specified.");
                  return PAR_INV;
                }
              }
              else
              {
                ERROR(
                    "Object name for object of type '"<< type <<"' was not specified.");
                return PAR_INV;
              }
            }
            else
            {
              std::string types;
              for (auto it = type_registry_.begin(); it != type_registry_.end();
                  it++)
              {
                types = types
                    + (it == type_registry_.begin() ?
                        std::string("'") : std::string(", '")) + (it->first)
                    + std::string("'");
              }
              ERROR(
                  "XML element '"<<type<<"' does not map to a known type for this factory! Supported types are:\n"<<types);
              return FAILURE;
            }
          }
          else
          {
            ERROR(
                "This factory can only handle std::string type of object idenfiers.");
            return PAR_INV;
          }
        }
        else
        {
          ERROR("Invalid XML handle");
          return PAR_ERR;
        }
      }
      ;

    private:
      /**
       * \brief Private Constructor
       */
      inline explicit Factory<I, BO>()
      {
      }
      ;

      /** The Map containing the register of the different types of classes **/
      std::map<I, BO * (*)()> type_registry_;
  };

  /**
   * \brief Registration Class for the object type: Also templated:
   * @param I   The Identifier type (typically string)
   * @param BO  The Base object type (required for the sake of the singleton factory)
   */
  template<typename I, typename BO>
  class Registrar
  {
    public:
      /**
       * \brief Public Constructor which is only used to register the new task type
       * @param name      The name for the new class type
       * @param creator   The creator function for the DERIVED class type but which returns a pointer to the base-class type!
       */
      Registrar(const I & name, BO * (*creator)())
      {
        Factory<I, BO>::Instance().registerType(name, creator);
      }
      ;
  };
}

#endif
