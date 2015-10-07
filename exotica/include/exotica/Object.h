/***********************************************************************\
|    Object is the base class from which all members of the exotica     |
 |   library should inherit. It currently provides functionality for     |
 |   run-time type identification, through C++ RTTI constructs. It is    |
 |   currently a header-only implementation.                             |
 |                                                                       |
 |           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
 |                    Last Edited: 12 - March - 2014                     |
 \***********************************************************************/

#ifndef EXOTICA_OBJECT_H
#define EXOTICA_OBJECT_H

#include <typeinfo> //!< The RTTI Functionality of C++
#include <cxxabi.h> //!< The demangler for gcc... this makes this system dependent!
#include <string>   //!< C++ type strings
#include <exotica/Tools.h>
#include <exotica/Server.h>
#include "rapidjson/document.h"

namespace exotica
{
  template<typename I, typename BO> class Factory;

  class Object
  {
      template<typename I, typename BO> friend class Factory;
    public:

      /**
       * \brief Constructor: default
       */
      Object()
          : ns_("")
      {
        //!< Empty constructor
      }

      virtual ~Object()
      {

      }

      /**
       * \brief Type Information wrapper: must be virtual so that it is polymorphic...
       * @return String containing the type of the object
       */
      inline virtual std::string type()
      {
        int status;
        std::string name;
        char * temp; //!< We need to store this to free the memory!

        temp = abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
        name = std::string(temp);
        free(temp);
        return name;
      }

      std::string getObjectName()
      {
        return object_name_;
      }

      virtual EReturn initBase(tinyxml2::XMLHandle & handle,
          const Server_ptr & server)
      {
        const char* atr = handle.ToElement()->Attribute("name");
        if (atr)
        {
          object_name_ = std::string(atr);
        }
        return SUCCESS;
      }

      virtual std::string print(std::string prepend)
      {
        return prepend + "  " + object_name_ + " (" + type() + ")";
      }

      //	Namespace, i.e. problem/scene/...etc
      std::string ns_;
      std::string object_name_;
  };
}
#endif
