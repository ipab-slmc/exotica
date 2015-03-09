#ifndef EXOTICA_CORE_TEST_H
#define EXOTICA_CORE_TEST_H

#include <gtest/gtest.h>
#include <exotica/EXOTica.hpp>
#include <boost/shared_ptr.hpp>

#include "testing_pkg/TestingTools.h"

namespace testing
{
  class DClass : public exotica::Object
  {
    public:
      DClass()
      {
        name = "TestingDerived";
      }
      std::string name;
  };
  
  EXOTICA_REGISTER(std::string, exotica::Object, "ObjectTest", exotica::Object);
  EXOTICA_REGISTER(std::string, exotica::Object, "DClassTest", testing::DClass);
  REGISTER_FOR_XML_TEST("DClassTest", "DerivedXML");
}


#endif
