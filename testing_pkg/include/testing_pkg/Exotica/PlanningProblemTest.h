#ifndef TESTING_PLANNING_PROBLEM_TEST_H
#define TESTING_PLANNING_PROBLEM_TEST_H

#include <exotica/EXOTica.hpp>
#include <tinyxml2/tinyxml2.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include "testing_pkg/TestingTools.h"
#include "testing_pkg/Exotica/DummyClasses.h"

class PlanningProblemTest : public ::testing::Test  //!< Testing class for the exotica library
{
  public: 
  
    virtual void SetUp()
    {
      ASSERT_EQ(exotica::SUCCESS, (exotica::PlanningProblem_fac::Instance().listImplementations(registered_types_)));  //!< Ensure that the object loads
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path));  //!< Removes dependency on ros (in the future)
      resource_path_ = package_path.append("/resource/Exotica/");
    };
    
    virtual void TearDown() {};
  
    //!< Member Variables
    std::vector<std::string>      registered_types_;
    std::string                   resource_path_;
    exotica::PlanningProblem_ptr  base_ptr_;    
};

#endif
