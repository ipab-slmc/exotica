#ifndef EXOTICA_TASK_TEST_H
#define EXOTICA_TASK_TEST_H

#include <gtest/gtest.h>
#include <exotica/EXOTica.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>

#include "testing_pkg/Exotica/TaskType_1.h"
#include "testing_pkg/Exotica/TaskType_2.h"
#include "testing_pkg/TestingTools.h"

typedef boost::shared_ptr<exotica::TaskDefinition> TaskPtr_t;

class ExoticaTaskTest : public ::testing::Test  //!< Testing class for the exotica library
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
    
    virtual void TearDown() {};
  
    //!< Member Variables
    std::vector<std::string>                registered_types_;
    exotica::OptimisationParameters_t       params_;
    boost::shared_ptr<ExoticaTaskTest_1>    task1_p_;
    boost::shared_ptr<ExoticaTaskTest_2>    task2_p_;
    std::string                             resource_path_;

};

#endif
