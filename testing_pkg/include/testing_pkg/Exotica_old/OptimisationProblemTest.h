#ifndef TESTING_OPTIMISATION_PROBLEM_TEST_H
#define TESTING_OPTIMISATION_PROBLEM_TEST_H

/**
 * To Test: 
 *  1) Initialisation (XML: manual is not necessary)
 *  2) Correct Jacobian/Weight/Error computation
 */
#include <exotica/EXOTica.hpp>
#include <gtest/gtest.h>
#include "testing_pkg/TestingTools.h"

class ExoticaOptProbTest : public ::testing::Test
{
  public:
  
    virtual void SetUp()
    {
      EXPECT_TRUE(exotica::TaskCreator::Instance()->listImplementations(registered_types_)) << "Could Not list the registered Objects";
      params_.optimisation_window = 1;
      std::string package_path;
      ASSERT_TRUE(findPackagePath("testing_pkg", package_path));  //!< Removes dependency on ros
      resource_path_ = package_path.append("/resource/Exotica/");
    };
    
    /** Data Members **/
    exotica::OptimisationProblem        problem;
    std::string                         resource_path_;
    std::vector<std::string>            registered_types_;
    exotica::OptimisationParameters_t   params_;
    
};

#endif
